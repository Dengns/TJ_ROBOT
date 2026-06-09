import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot


ROBOT_IP = "192.168.1.190"


def main():
    dcss = DCSS()
    robot = Marvin_Robot()

    if robot.connect(ROBOT_IP) == 0:
        raise RuntimeError(f"failed to connect robot at {ROBOT_IP}")

    print("Reading estimated Cartesian force and joint temperature. Ctrl+C to quit.")
    try:
        while True:
            sub_data = robot.subscribe(dcss)

            arm_a_force6 = sub_data["outputs"][0]["est_cart_fn"]
            arm_b_force6 = sub_data["outputs"][1]["est_cart_fn"]
            arm_a_temp = sub_data["outputs"][0]["fb_joint_them"]
            arm_b_temp = sub_data["outputs"][1]["fb_joint_them"]

            print(f"A force6={arm_a_force6} temp={arm_a_temp}")
            print(f"B force6={arm_b_force6} temp={arm_b_temp}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    finally:
        robot.release_robot()


if __name__ == "__main__":
    main()
