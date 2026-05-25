"""
Gripper controller with stall-detection grip sensing.

Mirrors Robotiq-style workflow:
  gripper.open()           — move to fully open
  gripper.close()          — close until object detected or fully closed
  gripper.is_holding       — True if an object was detected
  gripper.state            — GripperState enum
  gripper.on_state_changed — optional callback(GripperState)
"""
import time, threading
from enum import Enum


class GripperState(Enum):
    UNKNOWN = "unknown"
    OPEN    = "open"
    CLOSING = "closing"
    HOLDING = "holding"   # object detected mid-close


class Gripper:
    OPEN_ANGLE        =   0.0  # degrees, fully open
    CLOSE_ANGLE       = 100.0  # degrees, target when closing
    CLOSED_THRESHOLD  =   5.0  # degrees — if stall within this of CLOSE_ANGLE, no object
    STALL_THRESHOLD   =   2.0  # degrees of movement per interval to be considered moving
    STALL_COUNT       =   4    # consecutive still readings to confirm stall
    POLL_INTERVAL     =   0.05 # seconds between position checks
    OPEN_SPEED        =  60    # % speed for opening
    CLOSE_SPEED       =  35    # % speed for closing (gentler = better object feel)

    def __init__(self, driver, servo_id=6):
        self.driver    = driver
        self.servo_id  = servo_id
        self.state     = GripperState.UNKNOWN
        self._thread   = None
        self._stop     = threading.Event()
        self.on_state_changed = None  # callback(GripperState)

    # ------------------------------------------------------------------ public

    def open(self):
        """Open gripper fully and return immediately."""
        self._stop_monitor()
        self.driver.set_angle(self.servo_id, self.OPEN_ANGLE, speed_pct=self.OPEN_SPEED)
        self._set_state(GripperState.OPEN)

    def close(self, speed_pct=None):
        """
        Command close. Returns immediately.
        Monitors in background — state becomes HOLDING if object detected,
        or stays at CLOSE_ANGLE with state OPEN if nothing was gripped.
        """
        self._stop_monitor()
        self._stop.clear()
        speed = speed_pct or self.CLOSE_SPEED
        self.driver.set_angle(self.servo_id, self.CLOSE_ANGLE, speed_pct=speed)
        self._set_state(GripperState.CLOSING)
        self._thread = threading.Thread(target=self._monitor, daemon=True)
        self._thread.start()

    @property
    def is_holding(self):
        return self.state == GripperState.HOLDING

    @property
    def is_open(self):
        return self.state == GripperState.OPEN

    # ----------------------------------------------------------------- private

    def _read_angle(self):
        """Returns current gripper angle, or None on comms error."""
        pos = self.driver.get_position(self.servo_id)
        if pos == -1:
            return None
        return self.driver.get_angle(self.servo_id)

    def _monitor(self):
        """Background thread: watch for stall while closing."""
        history = []
        while not self._stop.is_set():
            time.sleep(self.POLL_INTERVAL)
            angle = self._read_angle()
            if angle is None:
                continue  # comms glitch, keep waiting

            history.append(angle)
            if len(history) > self.STALL_COUNT:
                history.pop(0)

            if len(history) == self.STALL_COUNT:
                spread = max(history) - min(history)
                if spread <= self.STALL_THRESHOLD:
                    # Stalled — if we stopped before reaching CLOSE_ANGLE, object is held
                    if angle < self.CLOSE_ANGLE - self.CLOSED_THRESHOLD:
                        self._set_state(GripperState.HOLDING)
                    else:
                        self._set_state(GripperState.OPEN)  # closed on air
                    return

    def _stop_monitor(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

    def _set_state(self, state):
        self.state = state
        if self.on_state_changed:
            self.on_state_changed(state)
