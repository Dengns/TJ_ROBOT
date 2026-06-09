import csv
import logging
import math
import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from SDK_PYTHON.fx_kine import FX_InvKineSolvePara, Marvin_Kine
from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS


logging.basicConfig(format="%(message)s")
logger = logging.getLogger("case10_cartesian_direct_cmd")
logger.setLevel(logging.DEBUG)

ROBOT_IP = "192.168.1.190"
CONFIG_PATH = os.path.join(current_dir, "ccs_m6_40.MvKDCfg")
CSV_PATH = os.path.join(current_dir, "cartesian_direct_cmd_log.csv")
PLOT_PATH = os.path.join(current_dir, "cartesian_direct_cmd_plot.png")


def check_joints_reached(current_joints, target_joints, tolerance=1.0):
    if len(current_joints) != 7 or len(target_joints) != 7:
        return False
    return all(abs(cur - tgt) < tolerance for cur, tgt in zip(current_joints, target_joints))


class CartesianRecorder:
    def __init__(self, kk, idx):
        self.kk = kk
        self.idx = idx
        self.t0 = None
        self.records = []
        self.events = []
        self.segment_id = 0
        self.target_pos = None
        self.prev_t = None
        self.prev_pos = None
        self.prev_vel = None

    def sample(self, sub_data):
        now = time.perf_counter()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        joints = sub_data["outputs"][self.idx]["fb_joint_pos"]
        low_speed_flag = sub_data["outputs"][self.idx]["low_speed_flag"][0]
        fk_mat = self.kk.fk(joints=joints)
        if not fk_mat:
            return

        pose = self.kk.mat4x4_to_xyzabc(pose_mat=fk_mat)
        pos = pose[:3]
        if self.target_pos is None:
            target_pos = [float("nan"), float("nan"), float("nan")]
            err = [float("nan"), float("nan"), float("nan")]
            err_norm = float("nan")
        else:
            target_pos = self.target_pos
            err = [target_pos[i] - pos[i] for i in range(3)]
            err_norm = math.sqrt(sum(value * value for value in err))

        if self.prev_t is None:
            vel = [0.0, 0.0, 0.0]
            acc = [0.0, 0.0, 0.0]
        else:
            dt = max(t - self.prev_t, 1e-6)
            vel = [(pos[i] - self.prev_pos[i]) / dt for i in range(3)]
            acc = [0.0, 0.0, 0.0] if self.prev_vel is None else [
                (vel[i] - self.prev_vel[i]) / dt for i in range(3)
            ]

        self.records.append(
            {
                "time_s": t,
                "x_mm": pos[0],
                "y_mm": pos[1],
                "z_mm": pos[2],
                "vx_mm_s": vel[0],
                "vy_mm_s": vel[1],
                "vz_mm_s": vel[2],
                "ax_mm_s2": acc[0],
                "ay_mm_s2": acc[1],
                "az_mm_s2": acc[2],
                "target_x_mm": target_pos[0],
                "target_y_mm": target_pos[1],
                "target_z_mm": target_pos[2],
                "err_x_mm": err[0],
                "err_y_mm": err[1],
                "err_z_mm": err[2],
                "err_norm_mm": err_norm,
                "low_speed_flag": low_speed_flag,
                "segment_id": self.segment_id,
            }
        )
        self.prev_t = t
        self.prev_pos = pos
        self.prev_vel = vel

    def mark_event(self, name, segment_id):
        event_time = 0.0 if self.t0 is None else time.perf_counter() - self.t0
        self.events.append({"time_s": event_time, "name": name, "segment_id": segment_id})

    def set_target_pose(self, target_pose):
        self.target_pos = target_pose[:3]

    def save_csv(self, path):
        if not self.records:
            logger.warning("no cartesian records to save")
            return

        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.records[0].keys()))
            writer.writeheader()
            writer.writerows(self.records)
        logger.info(f"saved cartesian csv: {path}")

    def save_plot(self, path):
        if not self.records:
            logger.warning("no cartesian records to plot")
            return

        try:
            import matplotlib.pyplot as plt
            from matplotlib.lines import Line2D
        except ImportError:
            logger.error("matplotlib is not installed; skipped plot generation")
            return

        t = [r["time_s"] for r in self.records]
        x = [r["x_mm"] for r in self.records]
        y = [r["y_mm"] for r in self.records]
        z = [r["z_mm"] for r in self.records]
        vx = [r["vx_mm_s"] for r in self.records]
        vy = [r["vy_mm_s"] for r in self.records]
        vz = [r["vz_mm_s"] for r in self.records]
        ax = [r["ax_mm_s2"] for r in self.records]
        ay = [r["ay_mm_s2"] for r in self.records]
        az = [r["az_mm_s2"] for r in self.records]
        err_x = [r["err_x_mm"] for r in self.records]
        err_y = [r["err_y_mm"] for r in self.records]
        err_z = [r["err_z_mm"] for r in self.records]
        err_norm = [r["err_norm_mm"] for r in self.records]

        series = {"X": (x, vx, ax), "Y": (y, vy, ay), "Z": (z, vz, az)}
        columns = [("Position", "mm"), ("Velocity", "mm/s"), ("Acceleration", "mm/s^2")]
        event_styles = {
            "command_sent": {"color": "tab:green", "linestyle": "--", "label": "joint command sent"},
            "motion_started": {"color": "tab:blue", "linestyle": "-.", "label": "motion started"},
            "segment_done": {"color": "tab:red", "linestyle": ":", "label": "segment done"},
        }

        fig, axes = plt.subplots(3, 3, figsize=(15, 10), sharex=True)
        for row, (axis_name, values) in enumerate(series.items()):
            for col, (title, unit) in enumerate(columns):
                plot_ax = axes[row][col]
                plot_ax.plot(t, values[col])
                plot_ax.set_title(f"{axis_name} {title}")
                plot_ax.set_ylabel(unit)
                plot_ax.grid(True)
                for event in self.events:
                    style = event_styles[event["name"]]
                    plot_ax.axvline(
                        event["time_s"],
                        color=style["color"],
                        linestyle=style["linestyle"],
                        alpha=0.45,
                    )
                if row == 2:
                    plot_ax.set_xlabel("Time (s)")

        if self.events:
            legend_ax = axes[0][0]
            for event in self.events:
                legend_ax.text(
                    event["time_s"],
                    0.98,
                    f"S{event['segment_id']} {event['name']}",
                    transform=legend_ax.get_xaxis_transform(),
                    rotation=90,
                    va="top",
                    ha="right",
                    fontsize=7,
                )

        legend_handles = [
            Line2D([0], [0], color=style["color"], linestyle=style["linestyle"], label=style["label"])
            for style in event_styles.values()
        ]
        fig.legend(handles=legend_handles, loc="upper center", ncol=3)
        fig.tight_layout()
        fig.subplots_adjust(top=0.92)
        fig.savefig(path, dpi=150)
        plt.close(fig)
        logger.info(f"saved cartesian plot: {path}")

        err_path = path.replace(".png", "_error.png")
        err_fig, err_axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        err_series = [
            ("X Error", err_x),
            ("Y Error", err_y),
            ("Z Error", err_z),
            ("3D Error Norm", err_norm),
        ]
        for err_ax, (title, values) in zip(err_axes, err_series):
            err_ax.plot(t, values)
            err_ax.axhline(1.0, color="tab:gray", linestyle="--", alpha=0.5, label="1 mm")
            err_ax.axhline(-1.0, color="tab:gray", linestyle="--", alpha=0.5)
            for event in self.events:
                style = event_styles[event["name"]]
                err_ax.axvline(
                    event["time_s"],
                    color=style["color"],
                    linestyle=style["linestyle"],
                    alpha=0.45,
                )
            err_ax.set_title(title)
            err_ax.set_ylabel("mm")
            err_ax.grid(True)
        err_axes[-1].set_xlabel("Time (s)")
        err_fig.legend(handles=legend_handles, loc="upper center", ncol=3)
        err_fig.tight_layout()
        err_fig.subplots_adjust(top=0.92)
        err_fig.savefig(err_path, dpi=150)
        plt.close(err_fig)
        logger.info(f"saved cartesian error plot: {err_path}")


def solve_ik(kk, target_pose, ref_joints):
    target_mat = kk.xyzabc_to_mat4x4(target_pose)
    if not target_mat:
        logger.error("--- target pose to matrix failed ---")
        return None

    sp = FX_InvKineSolvePara()
    sp.set_input_ik_target_tcp(kk.mat4x4_to_mat1x16(target_mat))
    sp.set_input_ik_ref_joint(ref_joints)

    result = kk.ik(structure_data=sp)
    if not result:
        logger.error("--- ik failed ---")
        return None
    if result.m_Output_IsOutRange or result.m_Output_IsJntExd:
        logger.error(
            f"--- ik unsafe: out_range={result.m_Output_IsOutRange}, "
            f"joint_exceed={result.m_Output_IsJntExd}, tags={list(result.m_Output_JntExdTags)} ---"
        )
        return None

    return result.m_Output_RetJoint.to_list()


def wait_joint_target(robot, kk, dcss, idx, target_joints, recorder, segment_id, timeout_s=10.0):
    start = time.perf_counter()
    start_pos = None
    motion_marked = False

    while True:
        sub_data = robot.subscribe(dcss)
        recorder.sample(sub_data)

        current_joints = sub_data["outputs"][idx]["fb_joint_pos"]
        current_record = recorder.records[-1]
        current_pos = [current_record["x_mm"], current_record["y_mm"], current_record["z_mm"]]
        if start_pos is None:
            start_pos = current_pos

        displacement = math.sqrt(sum((current_pos[i] - start_pos[i]) ** 2 for i in range(3)))
        if not motion_marked and displacement > 0.1:
            recorder.mark_event("motion_started", segment_id)
            motion_marked = True

        if check_joints_reached(current_joints, target_joints, tolerance=1.0):
            recorder.mark_event("segment_done", segment_id)
            return True

        if time.perf_counter() - start > timeout_s:
            logger.error("--- wait joint target timeout ---")
            recorder.mark_event("segment_done", segment_id)
            return False

        time.sleep(0.001)


def run_direct_segment(robot, kk, dcss, arm, idx, target_pose, recorder):
    recorder.segment_id += 1
    segment_id = recorder.segment_id

    sub_data = robot.subscribe(dcss)
    ref_joints = sub_data["outputs"][idx]["fb_joint_pos"]
    logger.info(f"segment {segment_id} ik ref joints: {ref_joints}")

    target_joints = solve_ik(kk, target_pose, ref_joints)
    if target_joints is None:
        return False
    logger.info(f"segment {segment_id} target joints: {target_joints}")
    recorder.set_target_pose(target_pose)

    if not robot.set_joint_position_cmd(arm=arm, joint=target_joints):
        logger.error("--- set joint target failed ---")
        return False

    recorder.mark_event("command_sent", segment_id)
    return wait_joint_target(robot, kk, dcss, idx, target_joints, recorder, segment_id)


def main():
    arm = "A"
    idx = 0 if arm == "A" else 1
    joint_vel_ratio = 20
    joint_acc_ratio = 20
    distance_mm = 20

    dcss = DCSS()
    robot = Concise_Marvin_Robot()

    if not robot.connect(robot_ip=ROBOT_IP, log_switch=0):
        logger.error("--- connect failed ---")
        return False

    try:
        kk = Marvin_Kine()
        kk.log_switch(0)
        ini_result = kk.load_config(arm_type=0, config_path=CONFIG_PATH)
        kk.initial_kine(
            robot_type=ini_result["TYPE"][0],
            dh=ini_result["DH"][0],
            pnva=ini_result["PNVA"][0],
            j67=ini_result["BD"][0],
        )

        if not robot.set_position_state(arm=arm, velRatio=joint_vel_ratio, AccRatio=joint_acc_ratio):
            logger.error("--- switch to position failed ---")
            return False
        time.sleep(0.5)

        sub_data = robot.subscribe(dcss)
        current_joints = sub_data["outputs"][idx]["fb_joint_pos"]
        logger.info(f"cartesian direct start joints: {current_joints}")

        fk_mat = kk.fk(joints=current_joints)
        if not fk_mat:
            logger.error("--- fk failed ---")
            return False

        pose_start = kk.mat4x4_to_xyzabc(pose_mat=fk_mat)
        logger.info(f"cartesian direct start pose: {pose_start}")
        recorder = CartesianRecorder(kk=kk, idx=idx)

        target_pose = pose_start.copy()
        target_pose[2] += distance_mm
        if not run_direct_segment(robot, kk, dcss, arm, idx, target_pose, recorder):
            return False

        target_pose = target_pose.copy()
        target_pose[1] -= distance_mm
        if not run_direct_segment(robot, kk, dcss, arm, idx, target_pose, recorder):
            return False

        target_pose = target_pose.copy()
        target_pose[2] -= distance_mm
        if not run_direct_segment(robot, kk, dcss, arm, idx, target_pose, recorder):
            return False

        target_pose = target_pose.copy()
        target_pose[1] += distance_mm
        if not run_direct_segment(robot, kk, dcss, arm, idx, target_pose, recorder):
            return False

        recorder.save_csv(CSV_PATH)
        recorder.save_plot(PLOT_PATH)
        return True
    finally:
        time.sleep(1)
        robot.disable(arm=arm)
        robot.release_robot()


if __name__ == "__main__":
    main()
