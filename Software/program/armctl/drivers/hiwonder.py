"""
Hiwonder HX-30HM Servo Driver

Protocol: half-duplex UART, packet [0x55][0x55][ID][Len][Cmd][Params...][Checksum]

Position encoding:
  READ  (SERVO_POS_READ=28): returns raw encoder  0–4095  (0–360° electrical)
  WRITE (SERVO_MOVE_TIME_WRITE=1): takes position 0–1000  (same 0–360° scale, lower res)
  Relation: write_pos = encoder × (1000 / 4096)

Angle conversion (per-joint, requires center_raw calibration):
  angle  = (encoder  − center_raw) × (360 / 4096)          # degrees from home
  encoder = center_raw + angle × (4096 / 360)
  write   = center_raw × (1000/4096) + angle × (1000/360)
"""
import time, struct, threading
from typing import Optional
from .base import ServoDriver, ServoState
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise ImportError("pyserial required: pip install pyserial")

# ── Protocol commands ──────────────────────────────────────────────────────────
SERVO_MOVE_TIME_WRITE      = 1
SERVO_MOVE_TIME_READ       = 2
SERVO_MOVE_TIME_WAIT_WRITE = 7
SERVO_MOVE_START           = 11
SERVO_MOVE_STOP            = 12
SERVO_ID_WRITE             = 13
SERVO_ID_READ              = 14
SERVO_ANGLE_OFFSET_ADJUST  = 17
SERVO_ANGLE_OFFSET_WRITE   = 18
SERVO_ANGLE_OFFSET_READ    = 19
SERVO_VIN_LIMIT_WRITE      = 22
SERVO_VIN_LIMIT_READ       = 23
SERVO_TEMP_MAX_LIMIT_WRITE = 24
SERVO_TEMP_MAX_LIMIT_READ  = 25
SERVO_TEMP_READ            = 26
SERVO_VIN_READ             = 27
SERVO_POS_READ             = 28
SERVO_OR_MOTOR_MODE_WRITE  = 29
SERVO_OR_MOTOR_MODE_READ   = 30
SERVO_LOAD_OR_UNLOAD_WRITE = 31
SERVO_LOAD_OR_UNLOAD_READ  = 32
SERVO_LED_CTRL_WRITE       = 33
SERVO_LED_CTRL_READ        = 34
SERVO_LED_ERROR_WRITE      = 35
SERVO_LED_ERROR_READ       = 36

BROADCAST_ID    = 254
ENCODER_MIN     = 0
ENCODER_MAX     = 4095
WRITE_POS_MIN   = 0
WRITE_POS_MAX   = 1000
DEFAULT_CENTER  = 2048


class HiwonderDriver(ServoDriver):
    def __init__(self, baudrate=1000000, timeout=0.1):
        self._baudrate = baudrate
        self._timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._centers: dict[int, int] = {}          # servo_id → center_raw
        self._write_offset: dict[int, float] = {}   # servo_id → write pos at angle=0°
        self._write_scale: dict[int, float] = {}    # servo_id → write units per degree

    # ── Connection ─────────────────────────────────────────────────────────────

    def connect(self, port=None):
        if port is None:
            port = self._auto_detect_port()
            if port is None:
                print("[HW] No port specified and auto-detect failed"); return False
        try:
            self._serial = serial.Serial(port=port, baudrate=self._baudrate,
                                         timeout=self._timeout, write_timeout=self._timeout)
            time.sleep(0.1); self._serial.reset_input_buffer()
            print(f"[HW] Connected on {port} at {self._baudrate} baud"); return True
        except serial.SerialException as e:
            print(f"[HW] Connection failed: {e}"); return False

    def disconnect(self):
        if self._serial and self._serial.is_open: self._serial.close()
        self._serial = None; print("[HW] Disconnected")

    def is_connected(self):
        return self._serial is not None and self._serial.is_open

    def configure_centers(self, centers: dict):
        """centers: {servo_id: (center_raw, enc_min, enc_max)}. Called by ArmController on connect."""
        for sid, val in centers.items():
            center_raw, enc_min, enc_max = val
            enc_range = enc_max - enc_min
            self._centers[sid]      = center_raw
            self._write_offset[sid] = (center_raw - enc_min) * 1000.0 / enc_range
            self._write_scale[sid]  = 4096.0 * 1000.0 / (enc_range * 360.0)

    def _center(self, servo_id: int) -> int:
        return self._centers.get(servo_id, DEFAULT_CENTER)

    def _auto_detect_port(self):
        for p in list_ports.comports():
            desc = (p.description or "").lower()
            mfg  = (p.manufacturer or "").lower()
            if any(x in desc for x in ["ch340","cp210","usb-serial","buslinker"]):
                print(f"[HW] Auto-detected: {p.device} ({p.description})"); return p.device
            if any(x in mfg for x in ["wch","silicon labs","hiwonder"]):
                print(f"[HW] Auto-detected: {p.device} ({p.manufacturer})"); return p.device
        return None

    # ── Low-level protocol ──────────────────────────────────────────────────────

    def _build_packet(self, servo_id, cmd, params=b""):
        length = len(params) + 3
        data = bytes([servo_id, length, cmd]) + params
        checksum = (~sum(data)) & 0xFF
        return bytes([0x55, 0x55]) + data + bytes([checksum])

    def _send(self, packet):
        if not self.is_connected(): raise ConnectionError("Not connected")
        self._serial.reset_input_buffer()
        self._serial.write(packet); self._serial.flush()

    def _receive(self, servo_id, cmd, timeout=0.1):
        if not self.is_connected(): return None
        deadline = time.time() + timeout; buf = bytearray()
        while time.time() < deadline:
            if self._serial.in_waiting > 0:
                buf.extend(self._serial.read(self._serial.in_waiting))
            else:
                time.sleep(0.001)
            while len(buf) >= 6:
                try: idx = buf.index(0x55)
                except ValueError: buf.clear(); break
                if idx > 0: buf = buf[idx:]
                if len(buf) < 2 or buf[1] != 0x55: buf = buf[1:]; continue
                if len(buf) < 4: break
                pkt_len = buf[3]; pkt_total = 3 + pkt_len
                if len(buf) < pkt_total: break
                packet = buf[:pkt_total]; buf = buf[pkt_total:]
                check_data = packet[2:-1]
                if packet[-1] != (~sum(check_data)) & 0xFF: continue
                if packet[2] == servo_id and packet[4] == cmd:
                    payload = bytes(packet[5:-1])
                    if payload:  # skip TX echo (same header/cmd but no data on half-duplex)
                        return payload
        return None

    def _write_cmd(self, servo_id, cmd, params=b""):
        with self._lock:
            self._send(self._build_packet(servo_id, cmd, params)); time.sleep(0.002)

    def _read_cmd(self, servo_id, cmd, timeout=0.1):
        with self._lock:
            self._send(self._build_packet(servo_id, cmd))
            time.sleep(0.003); return self._receive(servo_id, cmd, timeout)

    # ── Angle / position conversion ─────────────────────────────────────────────
    #
    # READ:  encoder (0–4095) = 0–360°
    # WRITE: write_pos (0–1000) = same 0–360° scale at lower resolution
    #        write_pos = encoder × (1000 / 4096)
    #
    # Per-joint logical angle (degrees from home):
    #   angle    = (encoder − center_raw) × (360 / 4096)
    #   encoder  = center_raw + angle × (4096 / 360)
    #   write_pos = center_raw × (1000/4096) + angle × (1000/360)

    def _encoder_to_angle(self, servo_id: int, encoder: int) -> float:
        return (encoder - self._center(servo_id)) * (360.0 / 4096.0)

    def _angle_to_write(self, servo_id: int, angle: float) -> int:
        offset = self._write_offset.get(servo_id, 500.0)
        scale  = self._write_scale.get(servo_id, 1000.0 / 240.0)
        return max(WRITE_POS_MIN, min(WRITE_POS_MAX, round(offset + angle * scale)))

    # ── ServoDriver interface ───────────────────────────────────────────────────

    def get_position(self, servo_id) -> int:
        """Returns raw encoder value (0–4095), or -1 on error."""
        resp = self._read_cmd(servo_id, SERVO_POS_READ)
        if resp and len(resp) >= 2:
            val = struct.unpack_from("<H", resp, 0)[0]  # unsigned 16-bit
            if ENCODER_MIN <= val <= ENCODER_MAX:
                return val
        return -1

    def set_position(self, servo_id, position, speed=0):
        """Write raw position (0–1000) with duration_ms."""
        position = max(WRITE_POS_MIN, min(WRITE_POS_MAX, position))
        speed = max(0, min(30000, speed))
        self._write_cmd(servo_id, SERVO_MOVE_TIME_WRITE, struct.pack("<HH", position, speed))

    def get_angle(self, servo_id) -> float:
        encoder = self.get_position(servo_id)
        if encoder == -1:
            return 0.0
        return self._encoder_to_angle(servo_id, encoder)

    def set_angle(self, servo_id, angle, speed_pct=50):
        pos = self._angle_to_write(servo_id, angle)
        duration_ms = 0 if speed_pct >= 100 else int(3000 * (1.0 - speed_pct / 100.0))
        self.set_position(servo_id, pos, duration_ms)

    def get_all_angles(self, servo_ids):
        angles = []
        for sid in servo_ids: angles.append(self.get_angle(sid)); time.sleep(0.003)
        return angles

    def set_all_angles(self, servo_ids, angles, speed_pct=50):
        for sid, a in zip(servo_ids, angles): self.set_angle(sid, a, speed_pct); time.sleep(0.002)

    def get_all_positions(self, servo_ids):
        positions = []
        for sid in servo_ids: positions.append(self.get_position(sid)); time.sleep(0.003)
        return positions

    def set_encoder(self, servo_id, encoder, speed_pct=50):
        write_pos = max(WRITE_POS_MIN, min(WRITE_POS_MAX, round(encoder * 1000.0 / 4096.0)))
        duration_ms = 0 if speed_pct >= 100 else int(3000 * (1.0 - speed_pct / 100.0))
        self.set_position(servo_id, write_pos, duration_ms)

    def set_all_encoders(self, servo_ids, encoders, speed_pct=50):
        for sid, enc in zip(servo_ids, encoders): self.set_encoder(sid, enc, speed_pct); time.sleep(0.002)

    def enable_torque(self, servo_id, enabled):
        self._write_cmd(servo_id, SERVO_LOAD_OR_UNLOAD_WRITE, bytes([1 if enabled else 0]))

    def set_damping_mode(self, servo_id, enabled, _damping=50):
        if enabled:
            self.enable_torque(servo_id, False)
        else:
            self.set_servo_mode(servo_id)
            self.enable_torque(servo_id, True)

    def set_motor_mode(self, servo_id, speed=0):
        speed = max(-1000, min(1000, speed))
        self._write_cmd(servo_id, SERVO_OR_MOTOR_MODE_WRITE,
                        bytes([1, 0]) + struct.pack("<h", speed))

    def set_servo_mode(self, servo_id):
        self._write_cmd(servo_id, SERVO_OR_MOTOR_MODE_WRITE, bytes([0, 0, 0, 0]))

    def read_temperature(self, servo_id):
        resp = self._read_cmd(servo_id, SERVO_TEMP_READ)
        return float(resp[0]) if resp and len(resp) >= 1 else -1.0

    def read_voltage(self, servo_id):
        resp = self._read_cmd(servo_id, SERVO_VIN_READ)
        return struct.unpack_from("<H", resp, 0)[0] / 1000.0 if resp and len(resp) >= 2 else -1.0

    def read_load_state(self, servo_id):
        resp = self._read_cmd(servo_id, SERVO_LOAD_OR_UNLOAD_READ)
        return resp[0] == 1 if resp and len(resp) >= 1 else False

    def get_state(self, servo_id):
        encoder = self.get_position(servo_id)
        angle = self._encoder_to_angle(servo_id, encoder) if encoder != -1 else 0.0
        return ServoState(id=servo_id, position=encoder,
                          angle=angle, speed=0.0, load=0.0,
                          voltage=self.read_voltage(servo_id),
                          temperature=self.read_temperature(servo_id),
                          torque_enabled=self.read_load_state(servo_id), moving=False)

    def get_all_states(self, servo_ids):
        return [self.get_state(s) for s in servo_ids]

    def ping(self, servo_id):
        return self.get_position(servo_id) >= 0

    def set_id(self, current_id, new_id):
        if not (0 <= new_id <= 253): return False
        self._write_cmd(current_id, SERVO_ID_WRITE, bytes([new_id]))
        time.sleep(0.1)
        ok = self.ping(new_id)
        print(f"[HW] ID change {'OK' if ok else 'FAILED'}: {current_id} -> {new_id}")
        return ok

    def scan(self, id_range=range(0, 20)):
        found = []
        for sid in id_range:
            if self.ping(sid): found.append(sid); print(f"[HW] Found servo ID {sid}")
        return found

    def read_id_broadcast(self):
        resp = self._read_cmd(BROADCAST_ID, SERVO_ID_READ, timeout=0.2)
        return resp[0] if resp and len(resp) >= 1 else None

    def set_led(self, servo_id, on):
        self._write_cmd(servo_id, SERVO_LED_CTRL_WRITE, bytes([0 if on else 1]))

    def move_stop(self, servo_id):
        self._write_cmd(servo_id, SERVO_MOVE_STOP)

    def move_all_to_center(self, servo_ids, duration_ms=1000):
        for sid in servo_ids: self.set_position(sid, 500, duration_ms); time.sleep(0.002)

    def unload_all(self, servo_ids):
        for sid in servo_ids: self.enable_torque(sid, False); time.sleep(0.002)

    def load_all(self, servo_ids):
        for sid in servo_ids: self.enable_torque(sid, True); time.sleep(0.002)
