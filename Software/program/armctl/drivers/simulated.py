import random, time
from .base import ServoDriver, ServoState

DEFAULT_CENTER = 2048  # encoder units, matches real hardware default

class SimulatedServo:
    def __init__(self, servo_id, center=DEFAULT_CENTER):
        self.id = servo_id
        self.center = center
        self.position = center          # starts at logical 0° (home)
        self.target_position = center
        self.speed = 0; self.torque_enabled = True; self.damping_mode = False
        self.temperature = 25.0 + random.uniform(0, 3); self.voltage = 12.0
        self.load = 0.0; self.last_update = time.time()

    def update(self):
        now = time.time(); dt = now - self.last_update; self.last_update = now
        if self.torque_enabled and not self.damping_mode:
            diff = self.target_position - self.position
            if abs(diff) > 0:
                step = max(-int(2000 * dt), min(int(2000 * dt), diff))
                self.position += step
                self.load = min(100, abs(diff) / 40.0)
            else:
                self.load = random.uniform(0, 3)

    @property
    def angle(self):
        return (self.position - self.center) * (360.0 / 4096.0)

    @angle.setter
    def angle(self, deg):
        self.target_position = max(0, min(4095, round(self.center + deg * (4096.0 / 360.0))))

    @property
    def moving(self):
        return abs(self.target_position - self.position) > 5


class SimulatedDriver(ServoDriver):
    def __init__(self):
        self._connected = False
        self._servos: dict[int, SimulatedServo] = {}
        self._centers: dict[int, int] = {}

    def connect(self, port=None):
        for i in range(1, 7):
            center = self._centers.get(i, DEFAULT_CENTER)
            self._servos[i] = SimulatedServo(i, center=center)
        self._connected = True
        print(f"[SIM] Connected ({len(self._servos)} servos)")
        return True

    def disconnect(self):
        self._connected = False; self._servos.clear()

    def is_connected(self):
        return self._connected

    def configure_centers(self, centers: dict):
        """centers: {servo_id: (center_raw, enc_min, enc_max)}"""
        for sid, val in centers.items():
            center_raw = val[0] if isinstance(val, tuple) else val
            self._centers[sid] = center_raw
            if sid in self._servos:
                self._servos[sid].center = center_raw

    def _get(self, sid):
        s = self._servos[sid]; s.update(); return s

    def get_position(self, sid):
        return self._get(sid).position

    def set_position(self, sid, pos, speed=0):
        s = self._get(sid)
        if s.torque_enabled:
            s.target_position = max(0, min(4095, pos))

    def get_angle(self, sid):
        return self._get(sid).angle

    def set_angle(self, sid, angle, speed_pct=50):
        s = self._get(sid)
        if s.torque_enabled:
            s.angle = angle

    def get_all_angles(self, sids):
        return [self.get_angle(s) for s in sids]

    def set_all_angles(self, sids, angles, speed_pct=50):
        for s, a in zip(sids, angles): self.set_angle(s, a, speed_pct)

    def get_all_positions(self, sids):
        return [self.get_position(s) for s in sids]

    def set_encoder(self, sid, encoder, speed_pct=50):
        s = self._get(sid)
        if s.torque_enabled:
            s.target_position = max(0, min(4095, round(encoder)))

    def set_all_encoders(self, sids, encoders, speed_pct=50):
        for s, enc in zip(sids, encoders): self.set_encoder(s, enc, speed_pct)

    def enable_torque(self, sid, enabled):
        s = self._get(sid); s.torque_enabled = enabled; s.damping_mode = False

    def set_damping_mode(self, sid, enabled, _damping=50):
        s = self._get(sid); s.damping_mode = enabled; s.torque_enabled = not enabled

    def get_state(self, sid):
        s = self._get(sid)
        return ServoState(s.id, s.position, s.angle, s.speed, s.load,
                          s.voltage, s.temperature, s.torque_enabled, s.moving)

    def get_all_states(self, sids):
        return [self.get_state(s) for s in sids]

    def ping(self, sid):
        return sid in self._servos

    def set_id(self, cur, new):
        if cur in self._servos:
            s = self._servos.pop(cur); s.id = new; self._servos[new] = s; return True
        return False
