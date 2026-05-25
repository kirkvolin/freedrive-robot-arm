import time, math, threading
from enum import Enum
from typing import List, Callable, Optional

class PlaybackState(Enum):
    IDLE = "idle"; RUNNING = "running"; PAUSED = "paused"; STOPPING = "stopping"

def interpolate_linear(start, end, t):
    t = max(0.0, min(1.0, t))
    return [s + (e - s) * t for s, e in zip(start, end)]

def trapezoidal_profile(t, duration, accel_frac=0.2):
    if duration <= 0: return 1.0
    t = max(0.0, min(duration, t))
    ta = duration * accel_frac; td = ta; tc = duration - ta - td
    if tc < 0: ta = td = duration/2; tc = 0
    if t < ta: return 0.5 * (t/ta)**2 * (1.0/(1.0-accel_frac))
    elif t < ta + tc:
        v_max = 1.0/(duration-ta); return 0.5*ta*v_max + v_max*(t-ta)
    else:
        t_d = t-ta-tc; v_max = 1.0/(duration-ta)
        return 0.5*ta*v_max + v_max*tc + v_max*t_d - 0.5*(t_d/td)**2*v_max*td

def compute_move_duration(start, end, speed_pct, max_speeds):
    if speed_pct <= 0: speed_pct = 1
    max_time = 0.0
    for s, e, ms in zip(start, end, max_speeds):
        dist = abs(e - s)
        if dist > 0.1: max_time = max(max_time, dist / (ms * speed_pct / 100.0))
    return max(0.1, max_time)

class TrajectoryPlayer:
    def __init__(self, move_callback, get_angles_callback, max_speeds=None):
        self.move_callback = move_callback; self.get_angles = get_angles_callback
        self.max_speeds = max_speeds or [80,60,80,100,100,120]
        self.state = PlaybackState.IDLE; self.current_step_index = 0; self.total_steps = 0
        self._thread = None; self._stop = threading.Event(); self._pause = threading.Event(); self._pause.set()
        self.on_state_change = None; self.on_step_change = None; self.on_complete = None

    def play(self, waypoint_sequence, loop=False):
        if self.state == PlaybackState.RUNNING: return
        self._stop.clear(); self._pause.set()
        self.total_steps = len(waypoint_sequence); self.current_step_index = 0
        self._thread = threading.Thread(target=self._loop, args=(waypoint_sequence, loop), daemon=True)
        self._thread.start(); self._set_state(PlaybackState.RUNNING)

    def pause(self):
        if self.state == PlaybackState.RUNNING: self._pause.clear(); self._set_state(PlaybackState.PAUSED)
    def resume(self):
        if self.state == PlaybackState.PAUSED: self._pause.set(); self._set_state(PlaybackState.RUNNING)
    def stop(self):
        self._stop.set(); self._pause.set()
        if self._thread and self._thread.is_alive(): self._thread.join(timeout=2.0)
        self._set_state(PlaybackState.IDLE)

    def _set_state(self, state):
        self.state = state
        if self.on_state_change: self.on_state_change(state)

    def _loop(self, seq, loop):
        try:
            while True:
                for step_idx, (target, spd, delay) in enumerate(seq):
                    if self._stop.is_set(): return
                    self._pause.wait(); self.current_step_index = step_idx
                    start = self.get_angles(); dur = compute_move_duration(start, target, spd, self.max_speeds)
                    dt = 1.0/50; t = 0.0
                    while t < dur:
                        if self._stop.is_set(): return
                        self._pause.wait()
                        self.move_callback(interpolate_linear(start, target, trapezoidal_profile(t, dur)))
                        time.sleep(dt); t += dt
                    self.move_callback(list(target))
                    if delay > 0:
                        end = time.time()
                        while time.time() - end < delay:
                            if self._stop.is_set(): return
                            time.sleep(0.05)
                if not loop: break
        finally:
            self._set_state(PlaybackState.IDLE)
            if self.on_complete: self.on_complete()
