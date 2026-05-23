"""
Arm Controller & Kinematics — Waypoint-based teach & playback
"""
import math, time, json, threading
import numpy as np
from typing import List, Tuple, Optional
from servo_interface import SimulatedServo
from dataclasses import dataclass, field, asdict

# ============================================================
# DH Forward Kinematics
# ============================================================

def dh_matrix(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])

DH_PARAMS = [
    {"d": 65.0,  "a": 0.0,   "alpha": math.pi/2},
    {"d": 0.0,   "a": 105.0, "alpha": 0.0},
    {"d": 0.0,   "a": 100.0, "alpha": 0.0},
    {"d": 0.0,   "a": 0.0,   "alpha": math.pi/2},
    {"d": 80.0,  "a": 0.0,   "alpha": 0.0},
]
THETA_OFFSETS = [0.0, math.pi/2, 0.0, 0.0, 0.0]

def forward_kinematics(angles_deg):
    angles_rad = [math.radians(a) for a in angles_deg[:5]]
    positions = [(0.0, 0.0, 0.0)]
    # Rx(-90°): makes Z_base = world Y so the d=65 base link rises vertically off the grid
    T = np.array([[1., 0., 0., 0.],
                  [0., 0., 1., 0.],
                  [0.,-1., 0., 0.],
                  [0., 0., 0., 1.]])
    transforms = []
    for i, (angle, params) in enumerate(zip(angles_rad, DH_PARAMS)):
        T = T @ dh_matrix(angle + THETA_OFFSETS[i], params["d"], params["a"], params["alpha"])
        transforms.append(T.copy()); positions.append((T[0,3], T[1,3], T[2,3]))
    return positions, transforms

def get_end_effector(angles_deg):
    positions, _ = forward_kinematics(angles_deg)
    return np.array(positions[-1])

def compute_jacobian(angles_deg, delta=0.1):
    ee = get_end_effector(angles_deg); J = np.zeros((3, 5))
    for i in range(5):
        ap = list(angles_deg[:5]); ap[i] += delta
        J[:, i] = (get_end_effector(ap) - ee) / delta
    return J

def inverse_kinematics(target, current_angles, max_iter=50, tolerance=1.0, damping=5.0):
    angles = list(current_angles[:5])
    limits = [(-150,150),(-90,90),(-120,120),(-100,100),(-150,150)]
    for _ in range(max_iter):
        ee = get_end_effector(angles); error = target - ee
        if np.linalg.norm(error) < tolerance:
            return angles + [current_angles[5] if len(current_angles) > 5 else 50]
        J = compute_jacobian(angles)
        d_theta = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), error)
        step_norm = np.max(np.abs(d_theta))
        if step_norm > 5.0: d_theta *= 5.0 / step_norm
        for i in range(5):
            angles[i] = max(limits[i][0], min(limits[i][1], angles[i] + d_theta[i]))
    return angles + [current_angles[5] if len(current_angles) > 5 else 50]


# ============================================================
# Waypoint System
# ============================================================

@dataclass
class Waypoint:
    name: str
    angles: List[float]
    speed_pct: int = 50
    delay_after: float = 0.0
    created_at: float = field(default_factory=time.time)

    def to_dict(self): return asdict(self)
    @classmethod
    def from_dict(cls, d): return cls(**d)

@dataclass
class Program:
    name: str
    waypoint_names: List[str] = field(default_factory=list)
    loop: bool = False

class WaypointManager:
    """Discrete waypoint storage with program sequencing — replaces TrajectoryRecorder."""

    def __init__(self):
        self.waypoints: dict[str, Waypoint] = {}
        self.program: List[str] = []  # ordered list of waypoint names
        self.counter = 0

    def save_waypoint(self, angles, name=None, speed_pct=50, delay_after=0.0):
        if name is None:
            self.counter += 1
            name = f"WP{self.counter:02d}"
        wp = Waypoint(name=name, angles=list(angles), speed_pct=speed_pct, delay_after=delay_after)
        self.waypoints[name] = wp
        self.program.append(name)
        return wp

    def delete_waypoint(self, name):
        if name in self.waypoints:
            del self.waypoints[name]
            self.program = [n for n in self.program if n != name]

    def move_step(self, from_idx, to_idx):
        if 0 <= from_idx < len(self.program) and 0 <= to_idx < len(self.program):
            self.program.insert(to_idx, self.program.pop(from_idx))

    def get_program_sequence(self):
        """Return ordered list of (waypoint, index) for playback."""
        result = []
        for name in self.program:
            wp = self.waypoints.get(name)
            if wp: result.append(wp)
        return result

    def clear_program(self):
        self.program.clear()

    def clear_all(self):
        self.waypoints.clear()
        self.program.clear()
        self.counter = 0

    @property
    def wp_count(self): return len(self.waypoints)

    @property
    def program_length(self): return len(self.program)

    def save_to_file(self, filepath):
        data = {
            "waypoints": {n: w.to_dict() for n, w in self.waypoints.items()},
            "program": self.program,
        }
        with open(filepath, "w") as f: json.dump(data, f, indent=2)

    def load_from_file(self, filepath):
        with open(filepath) as f: data = json.load(f)
        self.waypoints = {n: Waypoint.from_dict(w) for n, w in data["waypoints"].items()}
        self.program = data["program"]
        self.counter = len(self.waypoints)


# ============================================================
# Trajectory Interpolation
# ============================================================

def trapezoidal_profile(t, duration, accel_frac=0.2):
    if duration <= 0: return 1.0
    t = max(0.0, min(duration, t))
    ta = duration * accel_frac
    if ta * 2 > duration: ta = duration / 2
    tc = duration - 2 * ta
    v_max = 1.0 / (duration - ta) if (duration - ta) > 0 else 1.0
    if t < ta:
        return 0.5 * v_max * (t**2 / ta) if ta > 0 else 0.0
    elif t < ta + tc:
        return 0.5 * v_max * ta + v_max * (t - ta)
    else:
        t_d = t - ta - tc
        return 0.5 * v_max * ta + v_max * tc + v_max * t_d - 0.5 * v_max * (t_d**2 / ta) if ta > 0 else 1.0

def interpolate_linear(start, end, t):
    t = max(0.0, min(1.0, t))
    return [s + (e - s) * t for s, e in zip(start, end)]

def compute_move_duration(start, end, speed_pct):
    max_speeds = [80, 60, 80, 100, 100, 120]  # deg/sec per joint
    if speed_pct <= 0: speed_pct = 1
    max_time = 0.0
    for s, e, ms in zip(start, end, max_speeds):
        dist = abs(e - s)
        if dist > 0.1:
            max_time = max(max_time, dist / (ms * speed_pct / 100.0))
    return max(0.2, max_time)


# ============================================================
# Playback Engine
# ============================================================

class ProgramPlayer:
    """Plays a waypoint program with interpolated moves."""

    def __init__(self):
        self.playing = False
        self.paused = False
        self.current_step = 0
        self.move_progress = 0.0  # 0-1 within current move
        self.move_duration = 0.0
        self.move_elapsed = 0.0
        self.delay_remaining = 0.0
        self.start_angles = None
        self.target_angles = None
        self.sequence = []
        self.loop = False
        self.in_delay = False

    def start(self, sequence, start_angles, loop=False):
        if not sequence: return
        self.sequence = sequence
        self.loop = loop
        self.current_step = 0
        self.playing = True
        self.paused = False
        self._begin_move(start_angles, self.sequence[0])

    def stop(self):
        self.playing = False; self.paused = False
        self.current_step = 0; self.move_progress = 0.0

    def pause(self):
        if self.playing: self.paused = True

    def resume(self):
        if self.playing: self.paused = False

    def update(self, dt):
        """Advance playback, return interpolated angles or None."""
        if not self.playing or self.paused:
            return None

        # Handle post-move delay
        if self.in_delay:
            self.delay_remaining -= dt
            if self.delay_remaining <= 0:
                self.in_delay = False
                self._advance_step()
            return self.target_angles

        # Advance move
        self.move_elapsed += dt
        t = trapezoidal_profile(self.move_elapsed, self.move_duration)
        angles = interpolate_linear(self.start_angles, self.target_angles, t)

        if self.move_elapsed >= self.move_duration:
            # Move complete
            wp = self.sequence[self.current_step]
            if wp.delay_after > 0:
                self.in_delay = True
                self.delay_remaining = wp.delay_after
                return self.target_angles
            else:
                self._advance_step()
                return self.target_angles

        return angles

    def _advance_step(self):
        self.current_step += 1
        if self.current_step >= len(self.sequence):
            if self.loop:
                self.current_step = 0
                self._begin_move(self.target_angles, self.sequence[0])
            else:
                self.playing = False
        else:
            self._begin_move(self.target_angles, self.sequence[self.current_step])

    def _begin_move(self, from_angles, waypoint):
        self.start_angles = list(from_angles)
        self.target_angles = list(waypoint.angles)
        self.move_duration = compute_move_duration(self.start_angles, self.target_angles, waypoint.speed_pct)
        self.move_elapsed = 0.0
        self.move_progress = 0.0
        self.in_delay = False


# ============================================================
# Arm Controller
# ============================================================

class ArmController:
    def __init__(self):
        self.servos = [
            SimulatedServo(1, "J1 Base",       home_position=2048, min_angle=-150, max_angle=150),
            SimulatedServo(2, "J2 Shoulder",    home_position=2048, min_angle=-90,  max_angle=90),
            SimulatedServo(3, "J3 Elbow",       home_position=2048, min_angle=-120, max_angle=120),
            SimulatedServo(4, "J4 Wrist Pitch", home_position=2048, min_angle=-100, max_angle=100),
            SimulatedServo(5, "J5 Wrist Roll",  home_position=2048, min_angle=-150, max_angle=150),
            SimulatedServo(6, "J6 Gripper",     home_position=2048, min_angle=0,    max_angle=100),
        ]
        self.freedrive = False

    def get_joint_angles(self): return [s.get_angle() for s in self.servos]
    def set_joint_angle(self, index, angle):
        if 0 <= index < len(self.servos): self.servos[index].set_angle(angle)
    def set_all_angles(self, angles):
        for i, a in enumerate(angles):
            if i < len(self.servos): self.servos[i].set_angle(a)
    def home_all(self):
        for s in self.servos: s.set_position(s.home_position)
    def set_freedrive(self, enabled):
        self.freedrive = enabled
        for s in self.servos: s.set_damping(enabled)
    def get_end_effector_pos(self): return get_end_effector(self.get_joint_angles())
    def move_to_position(self, target):
        result = inverse_kinematics(target, self.get_joint_angles())
        if result:
            for i, a in enumerate(result): self.set_joint_angle(i, a)
            return True
        return False
    def get_fk_positions(self):
        positions, _ = forward_kinematics(self.get_joint_angles())
        return positions
    def get_all_status(self): return [s.get_status() for s in self.servos]
