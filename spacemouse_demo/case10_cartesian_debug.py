import logging
import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from SDK_PYTHON.fx_kine import Marvin_Kine
from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS


logging.basicConfig(format="%(message)s")
logger = logging.getLogger("case10_cartesian_debug")
logger.setLevel(logging.DEBUG)

ROBOT_IP = "192.168.1.190"
CONFIG_PATH = os.path.join(current_dir, "ccs_m6_40.MvKDCfg")


def wait_cart_pln_done(robot, dcss, idx):
    while True:
        data = robot.subscribe(dcss)
        if data["outputs"][idx]["traj_state"][0] == 0:
            break
        time.sleep(0.001)


def run_cart_segment(robot, kk, dcss, arm, idx, start_pose, end_pose, vel, acc):
    sub_data = robot.subscribe(dcss)
    current_joints = sub_data["outputs"][idx]["fb_joint_pos"]
    logger.info(f"segment ref joints: {current_joints}")

    points, pset = kk.movLA(
        start_xyzabc=start_pose,
        end_xyzabc=end_pose,
        ref_joints=current_joints,
        vel=vel,
        acc=acc,
        freq_hz=50,
    )
    logger.info(f"planned points: {len(points) if points else 0}")

    if not pset:
        logger.error("--- movLA failed ---")
        return False

    if not robot.run_pln_cart(arm=arm, pset=pset):
        logger.error("--- run cart pln failed ---")
        return False

    wait_cart_pln_done(robot, dcss, idx)
    return True


def main():
    arm = "A"
    idx = 0 if arm == "A" else 1
    cart_vel = 100
    cart_acc = 300
    distance_mm = 20

    dcss = DCSS()
    robot = Concise_Marvin_Robot()

    if not robot.connect(robot_ip=ROBOT_IP, log_switch=0):
        logger.error("--- connect failed ---")
        return False

    try:
        if not robot.pln_init(path=CONFIG_PATH):
            logger.error("--- initialize pln failed ---")
            return False

        kk = Marvin_Kine()
        kk.log_switch(0)
        ini_result = kk.load_config(arm_type=0, config_path=CONFIG_PATH)
        kk.initial_kine(
            robot_type=ini_result["TYPE"][0],
            dh=ini_result["DH"][0],
            pnva=ini_result["PNVA"][0],
            j67=ini_result["BD"][0],
        )

        sub_data = robot.subscribe(dcss)
        current_joints = sub_data["outputs"][idx]["fb_joint_pos"]
        logger.info(f"cartesian planning start joints: {current_joints}")

        fk_mat = kk.fk(joints=current_joints)
        if not fk_mat:
            logger.error("--- fk failed ---")
            return False

        pose_start = kk.mat4x4_to_xyzabc(pose_mat=fk_mat)
        logger.info(f"cartesian planning start pose: {pose_start}")

        # Four small Cartesian segments in the YZ plane:
        # Z+, Y-, Z-, Y+.
        pose_end = pose_start.copy()
        pose_end[2] += distance_mm
        if not run_cart_segment(robot, kk, dcss, arm, idx, pose_start, pose_end, cart_vel, cart_acc):
            return False

        pose_start = pose_end.copy()
        pose_end = pose_start.copy()
        pose_end[1] -= distance_mm
        if not run_cart_segment(robot, kk, dcss, arm, idx, pose_start, pose_end, cart_vel, cart_acc):
            return False

        pose_start = pose_end.copy()
        pose_end = pose_start.copy()
        pose_end[2] -= distance_mm
        if not run_cart_segment(robot, kk, dcss, arm, idx, pose_start, pose_end, cart_vel, cart_acc):
            return False

        pose_start = pose_end.copy()
        pose_end = pose_start.copy()
        pose_end[1] += distance_mm
        if not run_cart_segment(robot, kk, dcss, arm, idx, pose_start, pose_end, cart_vel, cart_acc):
            return False

        return True
    finally:
        time.sleep(1)
        robot.disable(arm=arm)
        robot.release_robot()


if __name__ == "__main__":
    main()
