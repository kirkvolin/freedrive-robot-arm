# Freedrive Teachable Robot Arm with GUI

A 6-DOF robot arm you teach by hand — grab it, guide it through positions, save waypoints, and build motion programs. No leader arm, no code, no joystick.

Built on the [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100) open-source platform with hardware and software modifications to support single-arm freedrive teaching without a leader arm.

## Why This Exists

The SO-ARM101 is designed for leader-follower teleoperation: you physically puppet a second "leader" arm to control the "follower." This works well for imitation learning pipelines like [LeRobot](https://github.com/huggingface/lerobot), but it means you need two arms, and the workflow is geared toward ML data collection rather than manual program building.

Industrial robots from Universal Robots, FANUC, and others offer a different interaction model — **freedrive mode** — where you grab the robot itself, guide it through positions by hand, and tap "save" at each point to build a program. No second arm, no code, no joystick. ArmCTL brings that workflow to a $200 desktop arm.

## Hardware Changes

### Motors: Hiwonder HX-30HM (replacing Feetech STS3215)

The SO-ARM101 normally uses Feetech STS3215 servos. The HX-30HM was chosen as a drop-in replacement for a few reasons:

- **12-bit magnetic encoder** (4096 counts/revolution) — wear-free, full 360° sensing, higher resolution than the STS3215's feedback
- **Motor mode** — the servo can be switched between position control and continuous rotation mode via software, which can theoretically be used for freedrive experimentation
- **30 kg·cm torque at 12V** — equivalent to the STS3215
- **Same 20×40mm form factor** — fits SO-ARM101 printed parts without modification, 25T spline on the output shaft

### Controller: Hiwonder BusLinker V3.0

USB-to-serial adapter for the Hiwonder servo bus. All six servos daisy-chain on a single half-duplex UART at 115200 baud. One board, one USB cable, one bus.

### Bill of Materials

| Part | Qty | ~Cost |
|------|-----|-------|
| Hiwonder HX-30HM servo | 6 | $90 |
| BusLinker V3.0 | 1 | $15 |
| 12V 5A power supply | 1 | $15 |
| 3D printed parts (PETG) | 1 set | $10-20 in filament |
| M2/M3 hardware, cables | misc | $10 |
| **Total** | | **~$130-150** |


### Freedrive Implementation

Freedrive mode disables servo torque and continuously reads joint positions via the magnetic encoders. The arm can be moved freely by hand while the software tracks exactly where it is. Press a key to snapshot the current joint angles as a named waypoint. Gravity compensation to counteract arm weight during freedrive is on the roadmap.

### Kinematics

Forward kinematics via DH parameter chain. Inverse kinematics using a damped least-squares Jacobian solver (~1.3mm accuracy). Supports both joint-space jogging and Cartesian jogging (X/Y/Z step moves resolved through IK).

### Motion Programs

Programs are ordered lists of waypoints with per-step speed and delay settings. Playback uses trapezoidal velocity profiles for smooth acceleration/deceleration. Programs save as JSON and support looping.

## Mechanical Design

The arm structure uses [SO-ARM101 follower STL files](https://github.com/TheRobotStudio/SO-ARM100/tree/main/STL/SO101/Individual) printed in PETG at 40%+ infill. The HX-30HM shares the STS3215 form factor, so the parts are dimensionally compatible.

**Print resources:**
- [SO-ARM101 STLs (GitHub)](https://github.com/TheRobotStudio/SO-ARM100/tree/main/STL/SO101/Individual)
- [MakerWorld plate layouts](https://makerworld.com/en/models/908660-so-arm101)
- [Assembly guide (Seeed Studio wiki)](https://wiki.seeedstudio.com/lerobot_so100m_new/)

## Roadmap

- [x] Hiwonder HX-30HM serial protocol driver
- [x] Forward/inverse kinematics engine
- [x] Trajectory interpolation with trapezoidal profiles
- [x] Waypoint and program system with JSON persistence
- [x] Hardware setup and test tooling
- [ ] PyQt6 desktop GUI (3D viewport, joint sliders, program builder)
- [ ] Cartesian jog buttons in GUI
- [ ] Gravity compensation for improved freedrive feel
- [ ] Custom arm redesign with longer links and stiffer joints
- [ ] ROS2/MoveIt integration for collision-aware motion planning

## Acknowledgments

- [The Robot Studio](https://github.com/TheRobotStudio) — SO-ARM100/101 open-source arm design
- [Hugging Face LeRobot](https://github.com/huggingface/lerobot) — robotics learning framework and community
- [Hiwonder](https://www.hiwonder.com/) — HX-30HM servos and BusLinker controller

## License

MIT
