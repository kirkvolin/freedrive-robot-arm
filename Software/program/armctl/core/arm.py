import time, threading
from ..config.arm_config import ArmConfig, DEFAULT_CONFIG
from ..drivers.base import ServoDriver
from ..core.kinematics import forward_kinematics, get_end_effector, inverse_kinematics
from ..core.waypoint import WaypointStore
from ..core.trajectory import TrajectoryPlayer, PlaybackState
from ..core.gripper import Gripper

class ArmMode:
    IDLE="idle"; JOG="jog"; FREEDRIVE="freedrive"; RUNNING="running"; FAULT="fault"

class ArmController:
    def __init__(self, driver: ServoDriver, config: ArmConfig = DEFAULT_CONFIG):
        self.driver = driver; self.config = config; self.store = WaypointStore(); self.mode = ArmMode.IDLE
        self._joint_ids = [j.id for j in config.joints]
        # max_speeds in encoder units/sec for trajectory timing
        self._max_speeds = [j.max_speed * (4096.0 / 360.0) for j in config.joints]
        self.player = TrajectoryPlayer(self._traj_move, self.get_joint_positions, self._max_speeds)
        self.player.on_state_change = self._on_pb_state
        self._poll_interval = 0.05; self._poll_thread = None; self._polling = False
        self._current_positions = [j.center_raw for j in config.joints]
        self._current_angles = [0.0] * config.num_joints; self._current_states = []
        self.on_angles_changed = None; self.on_mode_changed = None; self.on_state_updated = None
        self.gripper = Gripper(driver, servo_id=config.gripper_id)

    @property
    def _arm_joint_ids(self):
        return [j.id for j in self.config.joints if j.id != self.config.gripper_id]

    def connect(self, port=None):
        if not self.driver.connect(port): return False
        self.driver.configure_centers({j.id: (j.center_raw, j.enc_min, j.enc_max) for j in self.config.joints})
        self.store.load_all(); self._start_polling(); return True
    def disconnect(self): self._stop_polling(); self.player.stop(); self.driver.disconnect()
    def is_connected(self): return self.driver.is_connected()
    def get_joint_angles(self): return list(self._current_angles)
    def get_joint_positions(self): return list(self._current_positions)
    def get_joint_states(self): return list(self._current_states)
    def get_end_effector_pos(self): return get_end_effector(self._current_angles, self.config)
    def get_fk_positions(self):
        p, _ = forward_kinematics(self._current_angles, self.config); return p

    def move_joint(self, idx, angle, speed_pct=50):
        if self.mode == ArmMode.RUNNING: return
        jcfg = self.config.joints[idx]
        self.driver.set_angle(self._joint_ids[idx], max(jcfg.min_angle, min(jcfg.max_angle, angle)), speed_pct)
    def move_joints(self, angles, speed_pct=50):
        if self.mode == ArmMode.RUNNING: return
        clamped = [max(j.min_angle, min(j.max_angle, a)) for a, j in zip(angles, self.config.joints)]
        self.driver.set_all_angles(self._joint_ids, clamped, speed_pct)
    def move_to_position(self, target_xyz, speed_pct=50):
        result = inverse_kinematics(target_xyz, self._current_angles, self.config)
        if result: self.move_joints(result, speed_pct); return True
        return False
    def jog_joint(self, idx, delta): self.move_joint(idx, self._current_angles[idx] + delta)
    def jog_cartesian(self, axis, delta, speed_pct=50):
        t = self.get_end_effector_pos(); t[axis] += delta; self.move_to_position(t, speed_pct)
    def home(self): self.move_joints(self.config.home_angles, speed_pct=30)

    def set_freedrive(self, enabled):
        if enabled:
            self.player.stop()
            for jid in self._arm_joint_ids:
                self.driver.set_damping_mode(jid, True)
            self._set_mode(ArmMode.FREEDRIVE)
        else:
            for jid in self._arm_joint_ids:
                self.driver.set_damping_mode(jid, False)
            self._set_mode(ArmMode.IDLE)

    def save_waypoint(self, name, description=""): return self.store.add_waypoint(name, self._current_positions, description)
    def move_to_waypoint(self, name, speed_pct=50):
        wp = self.store.get_waypoint(name)
        if wp: self.driver.set_all_encoders(self._joint_ids, wp.positions, speed_pct); return True
        return False
    def delete_waypoint(self, name): self.store.remove_waypoint(name)
    def list_waypoints(self): return self.store.list_waypoints()

    def run_program(self, program_name):
        prog = self.store.get_program(program_name)
        if not prog or not prog.steps: return
        seq = []
        for step in prog.steps:
            wp = self.store.get_waypoint(step.waypoint_name)
            if wp: seq.append((wp.positions, step.speed_pct, step.delay_after))
        if seq: self._set_mode(ArmMode.RUNNING); self.player.play(seq, loop=prog.loop)
    def stop_program(self): self.player.stop(); self._set_mode(ArmMode.IDLE)
    def pause_program(self): self.player.pause()
    def resume_program(self): self.player.resume()

    def _traj_move(self, positions): self.driver.set_all_encoders(self._joint_ids, [round(p) for p in positions], speed_pct=100)
    def _on_pb_state(self, state):
        if state == PlaybackState.IDLE: self._set_mode(ArmMode.IDLE)
    def _set_mode(self, mode):
        self.mode = mode
        if self.on_mode_changed: self.on_mode_changed(mode)
    def _start_polling(self):
        self._polling = True
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True); self._poll_thread.start()
    def _stop_polling(self):
        self._polling = False
        if self._poll_thread: self._poll_thread.join(timeout=1.0)
    def _poll_loop(self):
        centers = [j.center_raw for j in self.config.joints]
        while self._polling and self.driver.is_connected():
            try:
                raw = self.driver.get_all_positions(self._joint_ids)
                # keep last good value for any joint that failed to read (-1)
                self._current_positions = [
                    r if r >= 0 else self._current_positions[i]
                    for i, r in enumerate(raw)
                ]
                self._current_angles = [
                    (pos - ctr) * (360.0 / 4096.0)
                    for pos, ctr in zip(self._current_positions, centers)
                ]
                self._current_states = self.driver.get_all_states(self._joint_ids)
                if self.on_angles_changed: self.on_angles_changed(self._current_angles)
                if self.on_state_updated: self.on_state_updated(self._current_states)
            except Exception as e: print(f"[POLL] Error: {e}")
            time.sleep(self._poll_interval)
