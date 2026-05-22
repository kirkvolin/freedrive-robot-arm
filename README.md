# 6-DOF Robot Arm Simulator

A Python-based 3D robot arm simulator with forward/inverse kinematics,
teach-and-playback, and a clean servo abstraction layer for swapping in
real Hiwonder HX-30HM hardware.

## Setup

```bash
# Create a virtual environment (recommended)
python -m venv venv
source venv/bin/activate   # Linux/Mac
# venv\Scripts\activate    # Windows

# Install dependencies
pip install -r requirements.txt
```

## Run

```bash
python main.py
```

## Controls

| Input                         | Action                              |
|-------------------------------|-------------------------------------|
| Left-drag on viewport         | Orbit camera                        |
| Right-drag anywhere           | IK drag (move end effector)         |
| Left-click near end effector  | IK drag (move end effector)         |
| Joint sliders                 | Direct joint angle control          |
| `H`                           | Home all joints                     |
| `F`                           | Toggle freedrive mode               |
| `R`                           | Start/stop recording trajectory     |
| `P`                           | Play/stop trajectory playback       |
| `Esc`                         | Quit                                |

## Project Structure

```
robot_arm/
├── main.py              # PyGame app: visualization, UI, event loop
├── arm_controller.py    # Kinematics (FK/IK), ArmController, TrajectoryRecorder
├── servo_interface.py   # Servo abstraction (SimulatedServo → HiwonderServo)
├── requirements.txt
└── README.md
```

## Architecture: Swapping in Real Hardware

The servo layer is designed for a clean swap. When your HX-30HM motors
arrive, you'll:

1. Create a `HiwonderServo` class in `servo_interface.py` with the same
   methods: `set_position()`, `get_angle()`, `set_damping()`, etc.
2. Open a serial connection to the BusLinker board.
3. Replace `SimulatedServo(...)` with `HiwonderServo(...)` in
   `arm_controller.py`.
4. Everything else (kinematics, UI, recording) stays identical.

## DH Parameters

The arm geometry is defined in `arm_controller.py` as DH parameters.
Adjust these to match your actual 3D-printed link lengths:

```python
DH_PARAMS = [
    {"d": 65.0,  "a": 0.0,   "alpha": π/2},   # Base height
    {"d": 0.0,   "a": 105.0, "alpha": 0.0},    # Upper arm length
    {"d": 0.0,   "a": 100.0, "alpha": 0.0},    # Forearm length
    {"d": 0.0,   "a": 0.0,   "alpha": π/2},    # Wrist
    {"d": 80.0,  "a": 0.0,   "alpha": 0.0},    # End effector offset
]
```

## Next Steps

- [ ] Implement HiwonderServo class with BusLinker serial protocol
- [ ] Add Cartesian jog buttons (X/Y/Z ± step)
- [ ] Waypoint save/load (JSON)
- [ ] Gravity compensation for better freedrive
- [ ] Simple program editor (sequence of waypoints + delays)
