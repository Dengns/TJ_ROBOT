"""Standalone Marvin joint-drag debug helper based on SDK showcase case7."""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from typing import Optional

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(CURRENT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import config as cfg
from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS

logger = logging.getLogger(__name__)


def _arm_to_index(arm: str) -> int:
    arm_upper = arm.upper()
    if arm_upper == "A":
        return 0
    if arm_upper == "B":
        return 1
    raise ValueError(f"unsupported arm: {arm!r}")


class JointDragDebugger:
    def __init__(
        self,
        ip: str,
        arm: str,
        timeout_s: float,
        exit_delay_s: float,
        poll_interval_s: float,
    ) -> None:
        self.ip = ip
        self.arm = arm.upper()
        self.arm_index = _arm_to_index(self.arm)
        self.timeout_s = max(0.0, timeout_s)
        self.exit_delay_s = max(0.0, exit_delay_s)
        self.poll_interval_s = max(0.001, poll_interval_s)
        self.robot: Optional[Concise_Marvin_Robot] = None
        self._dcss = DCSS()
        self._closed = False

    def setup(self) -> None:
        self.robot = Concise_Marvin_Robot()
        if not self.robot.connect(robot_ip=self.ip, log_switch=0):
            raise RuntimeError(f"failed to connect robot at {self.ip}")
        if not self.robot.set_joint_drag(arm=self.arm):
            raise RuntimeError(f"failed to switch arm {self.arm} to joint drag mode")
        time.sleep(0.5)
        logger.warning("=" * 72)
        logger.warning("Joint drag debug is active for arm %s.", self.arm)
        logger.warning("按住末端按钮开始拖拽；松开后可再次按住继续拖拽。")
        if self.timeout_s > 0.0:
            logger.warning("总超时 %.1f 秒，到时会自动关闭。", self.timeout_s)
        if self.exit_delay_s > 0.0:
            logger.warning("退出时会先等待 %.1f 秒，再关闭拖拽并下使能。", self.exit_delay_s)
        logger.warning("也可以按 Ctrl+C 立即退出。")
        logger.warning("=" * 72)

    def run(self) -> None:
        if self.robot is None:
            raise RuntimeError("call setup() before run()")

        start_mono = time.monotonic()
        drag_started = False
        tip_was_pressed = False

        while True:
            now_mono = time.monotonic()
            if self.timeout_s > 0.0 and now_mono - start_mono >= self.timeout_s:
                logger.warning("Reached drag session timeout %.1f s, closing.", self.timeout_s)
                return

            sub_data = self.robot.subscribe(self._dcss)
            if not sub_data:
                logger.warning("Subscribe failed; retrying.")
                time.sleep(self.poll_interval_s)
                continue

            tip_pressed = bool(sub_data["outputs"][self.arm_index]["tip_di"][0])

            if tip_pressed:
                if not drag_started:
                    drag_started = True
                    logger.info("Tip button pressed, drag started.")
                elif not tip_was_pressed:
                    logger.info("Tip button pressed again, continue dragging.")
            elif drag_started and tip_was_pressed:
                logger.info("Tip button released, session stays active for the next drag.")

            tip_was_pressed = tip_pressed

            time.sleep(self.poll_interval_s)

    def teardown(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.robot is not None:
            if self.exit_delay_s > 0.0:
                logger.info("Waiting %.1f s before closing drag session.", self.exit_delay_s)
                time.sleep(self.exit_delay_s)
            try:
                if not self.robot.exit_drag(arm=self.arm):
                    logger.warning("Exit drag failed for arm %s.", self.arm)
            except Exception as exc:
                logger.warning("Exit drag raised an error: %s", exc)
            try:
                self.robot.disable(arm=self.arm)
            except Exception as exc:
                logger.warning("Disable arm %s failed: %s", self.arm, exc)
            try:
                self.robot.release_robot()
            except Exception as exc:
                logger.warning("Release robot failed: %s", exc)


def _configure_logging(level_name: str) -> None:
    logging.basicConfig(level=getattr(logging, level_name), format="%(levelname)s %(name)s: %(message)s")
    logging.getLogger("debug_printer").setLevel(logging.WARNING)


def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone Marvin joint drag debug helper.")
    parser.add_argument("--ip", default=cfg.ROBOT_IP, help="robot IP address")
    parser.add_argument("--arm", default=cfg.ARM, choices=("A", "B"), help="arm to control")
    parser.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="auto-close session after this many seconds; use 0 to disable",
    )
    parser.add_argument(
        "--exit-delay",
        type=float,
        default=1.0,
        help="wait this many seconds during shutdown before exiting drag and disabling the arm",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.01,
        help="feedback polling interval in seconds",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"),
        help="logging level",
    )
    args = parser.parse_args()
    _configure_logging(args.log_level)

    session = JointDragDebugger(
        ip=args.ip,
        arm=args.arm,
        timeout_s=args.timeout,
        exit_delay_s=args.exit_delay,
        poll_interval_s=args.poll_interval,
    )

    def _sigint(_sig, _frame) -> None:
        logger.info("Ctrl+C received; shutting down joint drag debug.")
        session.teardown()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, _sigint)
    try:
        session.setup()
        session.run()
    finally:
        session.teardown()


if __name__ == "__main__":
    main()
