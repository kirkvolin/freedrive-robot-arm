"""
Arm Controller & Kinematics
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from servo_interface import SimulatedServo


# ============================================================
# DH Forward Kinematics
# ============================================================

def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Standard DH transformation matrix."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ],
    ])


# DH parameters for a 6-DOF arm approximating SO-ARM101 geometry
# Each row: (theta_offset, d, a, alpha) in mm and radians
DH_PARAMS = [
    # Joint 1: Base rotation (around Y axis)
    {"d": 65.0,  "a": 0.0,   "alpha": math.pi / 2},
    # Joint 2: Shoulder
    {"d": 0.0,   "a": 105.0, "alpha": 0.0},
    # Joint 3: Elbow
    {"d": 0.0,   "a": 100.0, "alpha": 0.0},
    # Joint 4: Wrist pitch
    {"d": 0.0,   "a": 0.0,   "alpha": math.pi / 2},
    # Joint 5: Wrist roll
    {"d": 80.0,  "a": 0.0,   "alpha": 0.0},
]

# Joint angle offsets 
THETA_OFFSETS = [0.0, math.pi / 2, 0.0, 0.0, 0.0]


def forward_kinematics(angles_deg: List[float]) -> List[np.ndarray]:

    angles_rad = [math.radians(a) for a in angles_deg[:5]]
    transforms = []
    positions = [(0.0, 0.0, 0.0)]  # Base origin

    T = np.eye(4)
    for i, (angle, params) in enumerate(zip(angles_rad, DH_PARAMS)):
        theta = angle + THETA_OFFSETS[i]
        T_joint = dh_matrix(theta, params["d"], params["a"], params["alpha"])
        T = T @ T_joint
        transforms.append(T.copy())
        positions.append((T[0, 3], T[1, 3], T[2, 3]))

    return positions, transforms


def get_end_effector(angles_deg: List[float]) -> np.ndarray:
    """Get end effector position as (x, y, z)."""
    positions, _ = forward_kinematics(angles_deg)
    return np.array(positions[-1])


# ============================================================
# Jacobian-based Inverse Kinematics
# ============================================================

def compute_jacobian(angles_deg: List[float], delta: float = 0.1) -> np.ndarray:

    ee = get_end_effector(angles_deg)
    J = np.zeros((3, 5))

    for i in range(5):
        angles_plus = list(angles_deg[:5])
        angles_plus[i] += delta
        ee_plus = get_end_effector(angles_plus)
        J[:, i] = (ee_plus - ee) / delta

    return J


def inverse_kinematics(target: np.ndarray, current_angles: List[float],
                       max_iter: int = 50, tolerance: float = 1.0,
                       damping: float = 5.0) -> Optional[List[float]]:

    angles = list(current_angles[:5])
    joint_limits = [
        (-150, 150),   # Base
        (-90, 90),     # Shoulder
        (-120, 120),   # Elbow
        (-100, 100),   # Wrist pitch
        (-150, 150),   # Wrist roll
    ]

    for iteration in range(max_iter):
        ee = get_end_effector(angles)
        error = target - ee
        dist = np.linalg.norm(error)

        if dist < tolerance:
            # Return full 6-angle list (preserve gripper)
            return angles + [current_angles[5] if len(current_angles) > 5 else 50]

        J = compute_jacobian(angles)
        # Damped least-squares: theta += J^T (J J^T + λ²I)^-1 * error
        JJT = J @ J.T + (damping ** 2) * np.eye(3)
        d_theta = J.T @ np.linalg.solve(JJT, error)

        # Scale step to avoid huge jumps
        max_step = 5.0  # max degrees per iteration
        step_norm = np.max(np.abs(d_theta))
        if step_norm > max_step:
            d_theta *= max_step / step_norm

        # Apply and clamp to joint limits
        for i in range(5):
            angles[i] += d_theta[i]
            angles[i] = max(joint_limits[i][0], min(joint_limits[i][1], angles[i]))

    # Return best attempt even if not converged
    return angles + [current_angles[5] if len(current_angles) > 5 else 50]


# ============================================================
# Arm Controller
# ============================================================

class ArmController:
    """Manages 6 servos as a coordinated arm."""

    def __init__(self):
        self.servos = [
            SimulatedServo(1, "J1 Base",        home_position=2048, min_angle=-150, max_angle=150),
            SimulatedServo(2, "J2 Shoulder",     home_position=2048, min_angle=-90,  max_angle=90),
            SimulatedServo(3, "J3 Elbow",        home_position=2048, min_angle=-120, max_angle=120),
            SimulatedServo(4, "J4 Wrist Pitch",  home_position=2048, min_angle=-100, max_angle=100),
            SimulatedServo(5, "J5 Wrist Roll",   home_position=2048, min_angle=-150, max_angle=150),
            SimulatedServo(6, "J6 Gripper",      home_position=2048, min_angle=0,    max_angle=100),
        ]
        self.freedrive = False

    def get_joint_angles(self) -> List[float]:
        return [s.get_angle() for s in self.servos]

    def set_joint_angle(self, index: int, angle: float):
        if 0 <= index < len(self.servos):
            self.servos[index].set_angle(angle)

    def home_all(self):
        for s in self.servos:
            s.set_position(s.home_position)

    def set_freedrive(self, enabled: bool):
        self.freedrive = enabled
        for s in self.servos:
            s.set_damping(enabled)

    def get_end_effector_pos(self) -> np.ndarray:
        return get_end_effector(self.get_joint_angles())

    def move_to_position(self, target: np.ndarray) -> bool:
        """Move end effector to target (x, y, z) using IK."""
        result = inverse_kinematics(target, self.get_joint_angles())
        if result is not None:
            for i, angle in enumerate(result):
                self.set_joint_angle(i, angle)
            return True
        return False

    def get_fk_positions(self) -> List[Tuple[float, float, float]]:
        """Get all joint positions for visualization."""
        positions, _ = forward_kinematics(self.get_joint_angles())
        return positions

    def get_all_status(self):
        return [s.get_status() for s in self.servos]


# ============================================================
# Trajectory Recorder
# ============================================================

class TrajectoryRecorder:
    """Records and plays back joint trajectories."""

    def __init__(self):
        self.frames: List[List[float]] = []
        self.recording = False
        self.record_interval = 0.1  # seconds
        self._last_record_time = 0

    def start_recording(self):
        self.frames = []
        self.recording = True
        self._last_record_time = 0

    def stop_recording(self):
        self.recording = False

    def record_frame(self, angles: List[float], current_time: float):
        if self.recording and current_time - self._last_record_time >= self.record_interval:
            self.frames.append(list(angles))
            self._last_record_time = current_time

    def get_frame(self, index: int) -> Optional[List[float]]:
        if 0 <= index < len(self.frames):
            return self.frames[index]
        return None

    @property
    def frame_count(self) -> int:
        return len(self.frames)

    @property
    def duration(self) -> float:
        return len(self.frames) * self.record_interval

    def clear(self):
        self.frames = []
        self.recording = False
