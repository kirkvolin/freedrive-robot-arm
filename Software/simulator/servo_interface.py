"""Servo Abstraction Layer"""
import math, random
from dataclasses import dataclass

@dataclass
class ServoStatus:
    id: int; position: int; angle: float; torque_enabled: bool
    temperature: float; voltage: float; load: float

class SimulatedServo:
    def __init__(self, servo_id, label="", home_position=2048, min_angle=-180, max_angle=180):
        self.id = servo_id; self.label = label or f"Joint {servo_id}"
        self.home_position = home_position
        self.min_position = 0; self.max_position = 4095
        self.min_angle = min_angle; self.max_angle = max_angle
        self.position = home_position; self.torque_enabled = True
        self.temperature = 25.0; self.voltage = 12.0; self.load = 0.0; self.damping_mode = False

    def position_to_angle(self, pos):
        return self.min_angle + ((pos - self.min_position) / (self.max_position - self.min_position)) * (self.max_angle - self.min_angle)
    def angle_to_position(self, angle):
        return int(self.min_position + ((angle - self.min_angle) / (self.max_angle - self.min_angle)) * (self.max_position - self.min_position))
    def set_position(self, pos):
        self.position = max(self.min_position, min(self.max_position, pos))
        self.load = random.uniform(0, 15); self.temperature = 25 + random.uniform(0, 8)
    def get_angle(self): return self.position_to_angle(self.position)
    def set_angle(self, angle): self.set_position(self.angle_to_position(max(self.min_angle, min(self.max_angle, angle))))
    def enable_torque(self, enabled): self.torque_enabled = enabled
    def set_damping(self, enabled, value=50): self.damping_mode = enabled; self.torque_enabled = not enabled
    def get_status(self): return ServoStatus(self.id, self.position, self.get_angle(), self.torque_enabled, self.temperature, self.voltage, self.load)
