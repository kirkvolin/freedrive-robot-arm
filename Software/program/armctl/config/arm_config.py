import math
from dataclasses import dataclass, field
from typing import List

@dataclass
class JointConfig:
    id: int; name: str; short_name: str
    min_angle: float; max_angle: float
    home_angle: float = 0.0; max_speed: float = 100.0; direction: int = 1
    center_raw: int = 2048  # encoder value (0-4095) at logical 0°
    enc_min: int = 0        # encoder value when servo is at write position 0 (physical min)
    enc_max: int = 4095     # encoder value when servo is at write position 1000 (physical max)

@dataclass
class ArmConfig:
    joints: List[JointConfig] = field(default_factory=lambda: [
        #                                                                center  enc_min  enc_max
        JointConfig(1, "Base",        "J1", -107,  116, 0.0,  80, center_raw=2048, enc_min=790,  enc_max=3400),
        JointConfig(2, "Shoulder",    "J2",  -90,   80, 0.0,  60, center_raw=3060, enc_min=2000, enc_max=4000),
        JointConfig(3, "Elbow",       "J3",  -87,   65, 0.0,  80, center_raw=1228, enc_min=200,  enc_max=2000),
        JointConfig(4, "Wrist Pitch", "J4",  -66,   77, 0.0, 100, center_raw=1790, enc_min=1000, enc_max=2700),
        JointConfig(5, "Wrist Roll",  "J5", -143,  150, 0.0, 100, center_raw=1860, enc_min=200,  enc_max=3600),
        JointConfig(6, "Gripper",     "J6",    0,  100, 50.0, 120, center_raw=2048, enc_min=0,   enc_max=4095),
    ])
    dh_params: list = field(default_factory=lambda: [
        {"d": 65.0,  "a": 0.0,   "alpha": math.pi/2, "offset": 0.0},
        {"d": 0.0,   "a": 105.0, "alpha": 0.0,        "offset": math.pi/2},
        {"d": 0.0,   "a": 100.0, "alpha": 0.0,        "offset": 0.0},
        {"d": 0.0,   "a": 0.0,   "alpha": math.pi/2,  "offset": 0.0},
        {"d": 80.0,  "a": 0.0,   "alpha": 0.0,        "offset": 0.0},
    ])
    gripper_id: int = 6
    baudrate: int = 1000000
    default_speed_pct: int = 50
    freedrive_damping: int = 50
    temperature_warning: float = 55.0
    temperature_fault: float = 70.0
    @property
    def num_joints(self): return len(self.joints)
    @property
    def home_angles(self): return [j.home_angle for j in self.joints]

DEFAULT_CONFIG = ArmConfig()
