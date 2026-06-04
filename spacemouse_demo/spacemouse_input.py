"""SpaceMouse reader using libspnav in a background polling thread."""

from __future__ import annotations

import ctypes
import math
import threading
import time
from typing import Tuple

import numpy as np

from config import DEADZONE, LIBSPNAV_PATH


def _quaternion_wxyz_from_axis_angle(rot_axis: np.ndarray, angle_rad: float) -> np.ndarray:
    norm = float(np.linalg.norm(rot_axis))
    if norm < 1e-12 or abs(angle_rad) < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    unit_axis = rot_axis / norm
    half_angle = angle_rad * 0.5
    sin_half = math.sin(half_angle)
    return np.array(
        [
            math.cos(half_angle),
            float(unit_axis[0]) * sin_half,
            float(unit_axis[1]) * sin_half,
            float(unit_axis[2]) * sin_half,
        ],
        dtype=np.float64,
    )


def _quaternion_wxyz_from_euler_intrinsic_xyz(
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array(
        [
            cr * cp * cy - sr * sp * sy,
            sr * cp * cy + cr * sp * sy,
            cr * sp * cy - sr * cp * sy,
            cr * cp * sy + sr * sp * cy,
        ],
        dtype=np.float64,
    )


def rotation_vector_from_quat_wxyz(q: np.ndarray) -> np.ndarray:
    """Log map from unit quaternion [w, x, y, z] to axis-angle vector."""
    w = float(np.clip(q[0], -1.0, 1.0))
    v = np.asarray(q[1:4], dtype=np.float64)
    v_norm = float(np.linalg.norm(v))
    if v_norm < 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(v_norm, w)
    return (v / v_norm) * angle


def quat_normalize_wxyz(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm


def quat_multiply_wxyz(q_left: np.ndarray, q_right: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = (float(q_left[0]), float(q_left[1]), float(q_left[2]), float(q_left[3]))
    w2, x2, y2, z2 = (float(q_right[0]), float(q_right[1]), float(q_right[2]), float(q_right[3]))
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def quat_conjugate_wxyz(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    return np.array([float(q[0]), -float(q[1]), -float(q[2]), -float(q[3])], dtype=np.float64)


def so3_rotation_vector_increment_between_quats_wxyz(
    q_current: np.ndarray,
    q_previous: np.ndarray,
) -> np.ndarray:
    qc = quat_normalize_wxyz(q_current)
    qp = quat_normalize_wxyz(q_previous)
    return rotation_vector_from_quat_wxyz(quat_multiply_wxyz(qc, quat_conjugate_wxyz(qp)))


def quat_from_rotation_vector_wxyz(omega: np.ndarray) -> np.ndarray:
    vector = np.asarray(omega, dtype=np.float64)
    theta = float(np.linalg.norm(vector))
    if theta < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    axis = vector / theta
    half_theta = theta * 0.5
    sin_half = math.sin(half_theta)
    return np.array(
        [math.cos(half_theta), float(axis[0]) * sin_half, float(axis[1]) * sin_half, float(axis[2]) * sin_half],
        dtype=np.float64,
    )


class MotionEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("z", ctypes.c_int),
        ("rx", ctypes.c_int),
        ("ry", ctypes.c_int),
        ("rz", ctypes.c_int),
        ("period", ctypes.c_uint),
        ("data", ctypes.c_void_p),
    ]


class ButtonEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("press", ctypes.c_int),
        ("bnum", ctypes.c_int),
    ]


class SpnavEvent(ctypes.Union):
    _fields_ = [
        ("type", ctypes.c_int),
        ("motion", MotionEvent),
        ("button", ButtonEvent),
    ]


SPNAV_EVENT_MOTION = 1
SPNAV_EVENT_BUTTON = 2


class SpaceMouseReader:
    """Non-blocking SpaceMouse reader using libspnav."""

    def __init__(self, lib_path: str = LIBSPNAV_PATH, deadzone: int = DEADZONE):
        self._lib = ctypes.CDLL(lib_path)
        self._deadzone = deadzone
        self._lock = threading.Lock()
        self._axes = [0, 0, 0, 0, 0, 0]
        self._buttons: dict[int, bool] = {}
        self._button_events: list[tuple[int, bool]] = []
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def open(self) -> None:
        if self._lib.spnav_open() == -1:
            raise RuntimeError("Cannot connect to spacenavd. Is the daemon running?")
        buf = ctypes.create_string_buffer(256)
        self._lib.spnav_dev_name(buf, 256)
        name = buf.value.decode(errors="replace")
        axes = self._lib.spnav_dev_axes()
        buttons = self._lib.spnav_dev_buttons()
        print(f"SpaceMouse connected: {name} (axes={axes}, buttons={buttons})")

    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._lib.spnav_close()
        print("SpaceMouse disconnected.")

    def get_axes(self) -> list[int]:
        with self._lock:
            return list(self._axes)

    def get_vec3_quaternion_wxyz(
        self,
        translation_scale: float = 1.0 / 350.0,
        *,
        rotation_mode: str = "axis_angle",
        rotation_rad_per_unit: float = math.pi / 350.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        with self._lock:
            axes = list(self._axes)
        x, y, z = float(axes[0]), float(axes[1]), float(axes[2])
        rx, ry, rz = float(axes[3]), float(axes[4]), float(axes[5])
        vec3 = np.array([x, y, z], dtype=np.float64) * translation_scale

        if rotation_mode == "axis_angle":
            rvec = np.array([rx, ry, rz], dtype=np.float64)
            q = _quaternion_wxyz_from_axis_angle(
                rvec,
                float(np.linalg.norm(rvec)) * rotation_rad_per_unit,
            )
        elif rotation_mode == "euler_xyz":
            q = _quaternion_wxyz_from_euler_intrinsic_xyz(
                rx * rotation_rad_per_unit,
                ry * rotation_rad_per_unit,
                rz * rotation_rad_per_unit,
            )
        else:
            raise ValueError(f"unsupported rotation_mode: {rotation_mode!r}")
        return vec3, quat_normalize_wxyz(q)

    def get_button(self, bnum: int) -> bool:
        with self._lock:
            return self._buttons.get(bnum, False)

    def pop_button_events(self) -> list[tuple[int, bool]]:
        with self._lock:
            events = list(self._button_events)
            self._button_events.clear()
            return events

    def _apply_deadzone(self, value: int) -> int:
        if abs(value) < self._deadzone:
            return 0
        return value - self._deadzone if value > 0 else value + self._deadzone

    def _poll_loop(self) -> None:
        ev = SpnavEvent()
        while not self._stop_event.is_set():
            ret = self._lib.spnav_poll_event(ctypes.byref(ev))
            if ret == 0:
                time.sleep(0.001)
                continue

            if ev.type == SPNAV_EVENT_MOTION:
                motion = ev.motion
                raw_axes = [motion.x, motion.y, motion.z, motion.rx, motion.ry, motion.rz]
                axes = [self._apply_deadzone(value) for value in raw_axes]
                with self._lock:
                    self._axes = axes
            elif ev.type == SPNAV_EVENT_BUTTON:
                button = ev.button
                pressed = bool(button.press)
                with self._lock:
                    self._buttons[button.bnum] = pressed
                    self._button_events.append((button.bnum, pressed))


if __name__ == "__main__":
    reader = SpaceMouseReader()
    reader.open()
    reader.start()
    print("Move SpaceMouse or press buttons. Ctrl+C to quit.")
    try:
        while True:
            axes = reader.get_axes()
            for bnum, pressed in reader.pop_button_events():
                print(f"\nButton {bnum} {'pressed' if pressed else 'released'}")
            print(
                f"\rT({axes[0]:5d},{axes[1]:5d},{axes[2]:5d}) "
                f"R({axes[3]:5d},{axes[4]:5d},{axes[5]:5d})",
                end="",
                flush=True,
            )
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    finally:
        reader.stop()
