"""SpaceMouse teleoperation for Marvin left/right arm in joint position mode."""

from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import sys
import time
from typing import Optional, Tuple

import numpy as np

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(CURRENT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import config as cfg
from spacemouse_input import (
    SpaceMouseReader,
    quat_from_rotation_vector_wxyz,
    quat_multiply_wxyz,
    quat_normalize_wxyz,
    rotation_vector_from_quat_wxyz,
    so3_rotation_vector_increment_between_quats_wxyz,
)
from SDK_PYTHON.fx_kine import FX_InvKineSolvePara, Marvin_Kine
from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS

logger = logging.getLogger(__name__)


def _arm_to_index(arm: str) -> int:
    arm_upper = arm.upper()
    if arm_upper == "A":
        return 0
    if arm_upper == "B":
        return 1
    raise ValueError(f"unsupported arm: {arm!r}")


def _rotmat_fk_to_quat_wxyz(fk_mat: list) -> np.ndarray:
    m00, m01, m02 = fk_mat[0][0], fk_mat[0][1], fk_mat[0][2]
    m10, m11, m12 = fk_mat[1][0], fk_mat[1][1], fk_mat[1][2]
    m20, m21, m22 = fk_mat[2][0], fk_mat[2][1], fk_mat[2][2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / scale
        x = (m21 - m12) * scale
        y = (m02 - m20) * scale
        z = (m10 - m01) * scale
    elif m00 > m11 and m00 > m22:
        scale = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale
    return quat_normalize_wxyz(np.array([w, x, y, z], dtype=np.float64))


def _quat_wxyz_to_rotmat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _tcp_mat4_from_position_quat_mm(position_m: np.ndarray, quat_wxyz: np.ndarray) -> list:
    rot = _quat_wxyz_to_rotmat(quat_normalize_wxyz(quat_wxyz))
    px, py, pz = (float(value) * 1000.0 for value in position_m)
    return [
        [float(rot[0, 0]), float(rot[0, 1]), float(rot[0, 2]), px],
        [float(rot[1, 0]), float(rot[1, 1]), float(rot[1, 2]), py],
        [float(rot[2, 0]), float(rot[2, 1]), float(rot[2, 2]), pz],
        [0.0, 0.0, 0.0, 1.0],
    ]


class CartesianPoseTarget:
    """Accumulated target TCP pose in robot base frame."""

    def __init__(self) -> None:
        self.position = np.zeros(3, dtype=np.float64)
        self.orientation_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def candidate_with_increment(
        self,
        delta_vec3_m: np.ndarray,
        delta_quat_wxyz: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        position = self.position + np.asarray(delta_vec3_m, dtype=np.float64)
        orientation = quat_normalize_wxyz(quat_multiply_wxyz(self.orientation_wxyz, delta_quat_wxyz))
        return position, orientation

    def set_pose(self, position_m: np.ndarray, quat_wxyz: np.ndarray) -> None:
        self.position = np.asarray(position_m, dtype=np.float64)
        self.orientation_wxyz = quat_normalize_wxyz(
            np.asarray(quat_wxyz, dtype=np.float64)
        )

    @staticmethod
    def is_position_in_workspace(position_m: np.ndarray) -> bool:
        return bool(
            np.all(np.asarray(position_m, dtype=np.float64) >= np.asarray(cfg.WORKSPACE_MIN, dtype=np.float64))
            and np.all(np.asarray(position_m, dtype=np.float64) <= np.asarray(cfg.WORKSPACE_MAX, dtype=np.float64))
        )


class SpatialIncrementComputer:
    """Map a SpaceMouse reading to a robot-frame pose increment."""

    def __init__(self) -> None:
        self._ema_translation = np.zeros(3, dtype=np.float64)
        self._ema_rotation = np.zeros(3, dtype=np.float64)
        self._meas_quat_prev = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self._trans_gain_m = 350.0 * cfg.TRANSLATION_SCALE

    def reset_rotation_measurement_anchor(self) -> None:
        self._meas_quat_prev = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def reset_filter_state(self) -> None:
        self._ema_translation = np.zeros(3, dtype=np.float64)
        self._ema_rotation = np.zeros(3, dtype=np.float64)
        self.reset_rotation_measurement_anchor()

    def compute(self, vec3_device: np.ndarray, quat_wxyz: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        mapped_translation = np.array(
            [vec3_device[cfg.AXIS_MAP[i]] * cfg.AXIS_SIGNS[i] for i in range(3)],
            dtype=np.float64,
        )
        alpha = cfg.EMA_ALPHA
        self._ema_translation = alpha * mapped_translation + (1.0 - alpha) * self._ema_translation

        omega_phys = self._so3_vector_from_measurement(quat_wxyz)
        kr = cfg.SM_QUAT_ROTATION_RAD_PER_UNIT
        rot_equiv = omega_phys / kr if kr > 1e-15 else np.zeros(3, dtype=np.float64)
        mapped_rotation = np.array(
            [rot_equiv[cfg.AXIS_MAP[3 + i] - 3] * cfg.AXIS_SIGNS[3 + i] for i in range(3)],
            dtype=np.float64,
        )
        self._ema_rotation = alpha * mapped_rotation + (1.0 - alpha) * self._ema_rotation

        trans_enable = np.array(cfg.AXIS_ENABLE[:3], dtype=np.float64)
        rot_enable = np.array(cfg.AXIS_ENABLE[3:6], dtype=np.float64)
        dpos = self._ema_translation * self._trans_gain_m * trans_enable
        dpos = np.clip(dpos, -cfg.MAX_TRANSLATION_PER_CYCLE, cfg.MAX_TRANSLATION_PER_CYCLE)
        omega_cmd = self._ema_rotation * cfg.ROTATION_SCALE * rot_enable
        omega_cmd = np.clip(omega_cmd, -cfg.MAX_ROTATION_PER_CYCLE, cfg.MAX_ROTATION_PER_CYCLE)
        return dpos.astype(np.float64), quat_normalize_wxyz(quat_from_rotation_vector_wxyz(omega_cmd))

    def _so3_vector_from_measurement(self, quat_wxyz: np.ndarray) -> np.ndarray:
        qn = quat_normalize_wxyz(quat_wxyz)
        if cfg.SM_ROT_SO3_SOURCE == "measurement_log":
            return rotation_vector_from_quat_wxyz(qn)
        if cfg.SM_ROT_SO3_SOURCE == "frame_relative_log":
            omega = so3_rotation_vector_increment_between_quats_wxyz(qn, self._meas_quat_prev)
            self._meas_quat_prev = np.copy(qn)
            return omega
        raise ValueError(f"unsupported SM_ROT_SO3_SOURCE: {cfg.SM_ROT_SO3_SOURCE!r}")


class TelemetryInterval:
    def __init__(self, interval_s: float) -> None:
        self._interval_s = max(0.0, interval_s)
        self._next_mono = 0.0

    def consume_if_due(self, now_mono: float) -> bool:
        if self._interval_s <= 0.0 or now_mono < self._next_mono:
            return False
        self._next_mono = now_mono + self._interval_s
        return True


class SpaceMouseMarvinTeleop:
    def __init__(
        self,
        ip: str,
        arm: str,
        config_path: str,
        control_rate_hz: int,
        telemetry_interval_s: float,
    ) -> None:
        self.ip = ip
        self.arm = arm.upper()
        self.arm_index = _arm_to_index(self.arm)
        self.config_path = config_path
        self.control_rate_hz = control_rate_hz
        self.pose = CartesianPoseTarget()
        self._increments = SpatialIncrementComputer()
        self._telemetry = TelemetryInterval(telemetry_interval_s)
        self._dcss = DCSS()
        self.mouse: Optional[SpaceMouseReader] = None
        self.robot: Optional[Concise_Marvin_Robot] = None
        self.kine: Optional[Marvin_Kine] = None
        self._closed = False

    def setup(self) -> None:
        self._setup_kinematics()
        self._setup_robot()
        self._setup_mouse()
        self._seed_pose_from_current_fk()
        logger.info("Teleop ready: arm=%s ip=%s rate=%sHz", self.arm, self.ip, self.control_rate_hz)

    def _setup_kinematics(self) -> None:
        self.kine = Marvin_Kine()
        self.kine.log_switch(0)
        ini = self.kine.load_config(arm_type=self.arm_index, config_path=self.config_path)
        if not ini:
            raise RuntimeError(f"failed to load kinematics config: {self.config_path}")
        ok = self.kine.initial_kine(
            robot_type=ini["TYPE"][self.arm_index],
            dh=ini["DH"][self.arm_index],
            pnva=ini["PNVA"][self.arm_index],
            j67=ini["BD"][self.arm_index],
        )
        if not ok:
            raise RuntimeError("failed to initialize Marvin kinematics")

    def _setup_robot(self) -> None:
        self.robot = Concise_Marvin_Robot()
        if not self.robot.connect(self.ip):
            raise RuntimeError(f"failed to connect robot at {self.ip}")
        if not self.robot.set_position_state(self.arm, cfg.VEL_RATIO, cfg.ACC_RATIO):
            raise RuntimeError("failed to enter joint position mode")
        sub_data = self._subscribe()
        if sub_data is None:
            raise RuntimeError("failed to subscribe robot feedback after connect")
        logger.info(
            "Robot state=%s err=%s vel=%s acc=%s",
            sub_data["states"][self.arm_index]["cur_state"],
            sub_data["states"][self.arm_index]["err_code"],
            sub_data["inputs"][self.arm_index]["joint_vel_ratio"],
            sub_data["inputs"][self.arm_index]["joint_acc_ratio"],
        )

    def _setup_mouse(self) -> None:
        self.mouse = SpaceMouseReader()
        self.mouse.open()
        self.mouse.start()
        self._increments.reset_rotation_measurement_anchor()

    def _seed_pose_from_current_fk(self) -> None:
        sub_data = self._subscribe()
        if sub_data is None:
            raise RuntimeError("cannot seed TCP pose because subscribe failed")
        joints = sub_data["outputs"][self.arm_index]["fb_joint_pos"]
        assert self.kine is not None
        fk_mat = self.kine.fk(joints)
        if not fk_mat:
            raise RuntimeError("cannot seed TCP pose because FK failed")
        self.pose.position = np.array(
            [fk_mat[0][3], fk_mat[1][3], fk_mat[2][3]],
            dtype=np.float64,
        ) * 1e-3
        self.pose.orientation_wxyz = quat_normalize_wxyz(_rotmat_fk_to_quat_wxyz(fk_mat))
        logger.info("Seeded target TCP from current FK: pos_m=%s", np.round(self.pose.position, 4).tolist())

    def teardown(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.mouse is not None:
            try:
                self.mouse.stop()
            except Exception as exc:
                logger.warning("SpaceMouse stop failed: %s", exc)
        if self.robot is not None:
            try:
                self.robot.disable(self.arm)
            except Exception as exc:
                logger.warning("Disable arm %s failed: %s", self.arm, exc)
            try:
                self.robot.release_robot()
            except Exception as exc:
                logger.warning("Release robot failed: %s", exc)

    def run(self) -> None:
        if self.mouse is None:
            raise RuntimeError("call setup() before run()")
        period_s = 1.0 / float(self.control_rate_hz)
        while True:
            t0 = time.monotonic()
            if not self.mouse.get_button(cfg.DEADMAN_BUTTON):
                self._increments.reset_filter_state()
                self._sleep_until_period(t0, period_s)
                continue
            vec3, quat_wxyz = self.mouse.get_vec3_quaternion_wxyz(
                translation_scale=cfg.SM_VEC3_TRANSLATION_SCALE,
                rotation_mode=cfg.SM_QUAT_ROTATION_MODE,
                rotation_rad_per_unit=cfg.SM_QUAT_ROTATION_RAD_PER_UNIT,
            )
            dvec, dq = self._increments.compute(vec3, quat_wxyz)
            candidate_position, candidate_orientation = self.pose.candidate_with_increment(dvec, dq)
            if not self.pose.is_position_in_workspace(candidate_position):
                logger.warning(
                    "Workspace limit reached; stop command. target_m=%s",
                    np.round(candidate_position, 4).tolist(),
                )
                self._sleep_until_period(t0, period_s)
                continue
            self.pose.set_pose(candidate_position, candidate_orientation)
            self._maybe_log_telemetry(vec3, quat_wxyz, dvec, dq, time.monotonic())
            self._send_ik_position_command()
            self._sleep_until_period(t0, period_s)

    def _send_ik_position_command(self) -> None:
        sub_data = self._subscribe()
        if sub_data is None:
            logger.warning("Subscribe failed; skip this cycle.")
            return
        ref_joints = sub_data["outputs"][self.arm_index]["fb_joint_pos"]
        target_mat = _tcp_mat4_from_position_quat_mm(self.pose.position, self.pose.orientation_wxyz)

        assert self.kine is not None
        sp = FX_InvKineSolvePara()
        sp.set_input_ik_target_tcp(self.kine.mat4x4_to_mat1x16(target_mat))
        sp.set_input_ik_ref_joint(ref_joints)
        result = self.kine.ik(sp)
        if not result:
            logger.warning("IK failed; skip this cycle.")
            return
        if result.m_Output_IsOutRange or result.m_Output_IsJntExd or any(result.m_Output_IsDeg):
            logger.warning(
                "IK rejected: out_range=%s jnt_exd=%s deg=%s tags=%s",
                bool(result.m_Output_IsOutRange),
                bool(result.m_Output_IsJntExd),
                [bool(value) for value in result.m_Output_IsDeg],
                [bool(value) for value in result.m_Output_JntExdTags],
            )
            return

        joint_cmd = [float(value) for value in result.m_Output_RetJoint.to_list()]
        assert self.robot is not None
        if not self.robot.set_joint_position_cmd(self.arm, joint_cmd):
            logger.warning("SetJointPostionCmd failed; skip this cycle.")

    def _subscribe(self) -> dict | None:
        assert self.robot is not None
        return self.robot.subscribe(self._dcss)

    def _maybe_log_telemetry(
        self,
        vec3: np.ndarray,
        quat_wxyz: np.ndarray,
        dvec: np.ndarray,
        dq: np.ndarray,
        now_mono: float,
    ) -> None:
        if not self._telemetry.consume_if_due(now_mono):
            return
        logger.info(
            "SpaceMouse vec=%s q=%s dpos_mm=%s dq=%s target_m=%s",
            np.round(vec3, 4).tolist(),
            np.round(quat_wxyz, 4).tolist(),
            np.round(dvec * 1000.0, 4).tolist(),
            np.round(dq, 4).tolist(),
            np.round(self.pose.position, 4).tolist(),
        )

    @staticmethod
    def _sleep_until_period(t0: float, period_s: float) -> None:
        sleep_s = period_s - (time.monotonic() - t0)
        if sleep_s > 0:
            time.sleep(sleep_s)


def _configure_logging(level_name: str) -> None:
    logging.basicConfig(level=getattr(logging, level_name), format="%(levelname)s %(name)s: %(message)s")


def main() -> None:
    parser = argparse.ArgumentParser(description="SpaceMouse teleop for Marvin joint position mode.")
    parser.add_argument("--ip", default=cfg.ROBOT_IP, help="robot IP address")
    parser.add_argument("--arm", default=cfg.ARM, choices=("A", "B"), help="arm to control")
    parser.add_argument("--config", default=cfg.CONFIG_PATH, help="Marvin kinematics .MvKDCfg path")
    parser.add_argument("--rate", type=int, default=cfg.CONTROL_RATE_HZ, help="control rate in Hz")
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"),
        help="logging level",
    )
    parser.add_argument(
        "--print-mouse",
        action="store_true",
        help="print SpaceMouse telemetry periodically",
    )
    args = parser.parse_args()
    _configure_logging(args.log_level)

    telemetry_interval = cfg.TELEMETRY_INTERVAL_S if args.print_mouse else 0.0
    teleop = SpaceMouseMarvinTeleop(args.ip, args.arm, args.config, args.rate, telemetry_interval)

    def _sigint(_sig, _frame) -> None:
        logger.info("Ctrl+C received; shutting down.")
        teleop.teardown()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, _sigint)
    try:
        teleop.setup()
        teleop.run()
    finally:
        teleop.teardown()


if __name__ == "__main__":
    main()
