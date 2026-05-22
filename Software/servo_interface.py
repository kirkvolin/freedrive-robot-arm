
import math
import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ServoStatus:
    id: int
    position: int          # 0-4095 (12-bit encoder)
    angle: float           # degrees
    torque_enabled: bool
    temperature: float     # °C
    voltage: float         # V
    load: float            # % of max


class SimulatedServo:
    """Simulated servo matching HX-30HM behavior."""

    def __init__(self, servo_id: int, label: str = "",
                 home_position: int = 2048,
                 min_angle: float = -180, max_angle: float = 180):
        self.id = servo_id
        self.label = label or f"Joint {servo_id}"
        self.home_position = home_position
        self.min_position = 0
        self.max_position = 4095
        self.min_angle = min_angle
        self.max_angle = max_angle

        self.position = home_position
        self.torque_enabled = True
        self.temperature = 25.0
        self.voltage = 12.0
        self.load = 0.0
        self.damping_mode = False

    def position_to_angle(self, pos: int) -> float:
        pos_range = self.max_position - self.min_position
        angle_range = self.max_angle - self.min_angle
        return self.min_angle + ((pos - self.min_position) / pos_range) * angle_range

    def angle_to_position(self, angle: float) -> int:
        pos_range = self.max_position - self.min_position
        angle_range = self.max_angle - self.min_angle
        return int(self.min_position + ((angle - self.min_angle) / angle_range) * pos_range)

    def set_position(self, pos: int):
        self.position = max(self.min_position, min(self.max_position, pos))
        # Simulate load/temp changes
        import random
        self.load = random.uniform(0, 15)
        self.temperature = 25 + random.uniform(0, 8)

    def get_angle(self) -> float:
        return self.position_to_angle(self.position)

    def set_angle(self, angle: float):
        clamped = max(self.min_angle, min(self.max_angle, angle))
        self.set_position(self.angle_to_position(clamped))

    def enable_torque(self, enabled: bool):
        self.torque_enabled = enabled

    def set_damping(self, enabled: bool, value: int = 50):
        """HX-30HM damping mode for freedrive."""
        self.damping_mode = enabled
        self.torque_enabled = not enabled

    def get_status(self) -> ServoStatus:
        return ServoStatus(
            id=self.id,
            position=self.position,
            angle=self.get_angle(),
            torque_enabled=self.torque_enabled,
            temperature=self.temperature,
            voltage=self.voltage,
            load=self.load,
        )


# ============================================================
# Future: HiwonderServo 
# ============================================================
#
# class HiwonderServo:
#     """HX-30HM servo via BusLinker serial."""
#
#     def __init__(self, servo_id: int, bus, label: str = "", ...):
#         self.id = servo_id
#         self.bus = bus  # serial.Serial connection to BusLinker
#         self.label = label
#         ...
#
#     def set_position(self, pos: int):
#         # Send position command via Hiwonder protocol
#         packet = self._build_packet(CMD_SERVO_MOVE, self.id, pos)
#         self.bus.write(packet)
#
#     def get_angle(self) -> float:
#         # Read position from servo
#         packet = self._build_packet(CMD_POS_READ, self.id)
#         self.bus.write(packet)
#         response = self.bus.read(...)
#         return self._parse_angle(response)
#
#     def set_damping(self, enabled: bool, value: int = 50):
#         # HX-30HM damping mode command
#         ...
