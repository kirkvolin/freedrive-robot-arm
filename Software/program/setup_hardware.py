"""
Hardware Setup & Test Script for HX-30HM servos.

Usage:
    python setup_hardware.py scan                  # Find connected servos
    python setup_hardware.py --port COM3 scan      # Specify port
    python setup_hardware.py set_id 1 2            # Change servo ID 1 -> 2
    python setup_hardware.py set_id -1 1           # Auto-detect then set to 1
    python setup_hardware.py test                  # Test all servos
    python setup_hardware.py test_one 3            # Test servo ID 3
    python setup_hardware.py freedrive             # Freedrive mode (read angles)
    python setup_hardware.py center                # All servos to center
    python setup_hardware.py interactive           # Python shell with driver
"""
import sys, time, argparse
sys.path.insert(0, ".")
from armctl.drivers.hiwonder import HiwonderDriver


def find_and_connect(port=None):
    driver = HiwonderDriver()
    if driver.connect(port):
        return driver
    print("\nCould not connect to BusLinker.")
    print("  1. Is the BusLinker plugged in via USB?")
    print("  2. Is the power supply connected and on?")
    print("  3. Check Device Manager (Windows) or ls /dev/tty* (Linux)")
    print("  4. Try: python setup_hardware.py --port COM3 scan")
    sys.exit(1)


def cmd_scan(driver):
    print("\n=== Scanning for servos (ID 0-19) ===\n")
    found = driver.scan(range(0, 20))
    if found:
        print(f"\nFound {len(found)} servo(s): {found}\n")
        for sid in found:
            state = driver.get_state(sid)
            print(f"  ID {sid}: pos={state.position}, angle={state.angle:.1f}, "
                  f"temp={state.temperature:.0f}C, voltage={state.voltage:.1f}V, "
                  f"torque={'ON' if state.torque_enabled else 'OFF'}")
    else:
        print("No servos found! Check wiring and power.")


def cmd_set_id(driver, current_id, new_id):
    print(f"\n=== Changing servo ID: {current_id} -> {new_id} ===")
    print("WARNING: Only ONE servo should be connected!\n")
    if current_id == -1:
        print("Attempting broadcast ID read...")
        detected = driver.read_id_broadcast()
        if detected is not None:
            print(f"Detected servo at ID {detected}")
            current_id = detected
        else:
            print("Could not detect servo via broadcast"); return
    input(f"Press Enter to change ID {current_id} -> {new_id} (Ctrl+C to cancel)...")
    if driver.set_id(current_id, new_id):
        print(f"Success! Servo responds at ID {new_id}" if driver.ping(new_id) else "Changed but no response")
    else:
        print("Failed to change ID")


def cmd_test(driver, servo_ids=None):
    if servo_ids is None:
        print("\n=== Scanning ===")
        servo_ids = driver.scan(range(0, 10))
        if not servo_ids: print("No servos found!"); return
    print(f"\n=== Testing servos: {servo_ids} ===\n")
    for sid in servo_ids:
        print(f"--- Servo {sid} ---")
        state = driver.get_state(sid)
        print(f"  Status: pos={state.position}, angle={state.angle:.1f}, "
              f"temp={state.temperature:.0f}C, voltage={state.voltage:.1f}V")
        print(f"  Moving to center...", end="", flush=True)
        driver.set_position(sid, 500, 1000); time.sleep(1.2)
        print(f" pos={driver.get_position(sid)}")
        print(f"  Moving +30deg...", end="", flush=True)
        driver.set_position(sid, 625, 500); time.sleep(0.7)
        print(f" pos={driver.get_position(sid)}")
        print(f"  Moving -30deg...", end="", flush=True)
        driver.set_position(sid, 375, 500); time.sleep(0.7)
        print(f" pos={driver.get_position(sid)}")
        print(f"  Back to center...", end="", flush=True)
        driver.set_position(sid, 500, 500); time.sleep(0.7)
        print(f" done\n")
    print("All tests complete!")


def cmd_freedrive(driver):
    print("\n=== Scanning ===")
    servo_ids = driver.scan(range(0, 10))
    if not servo_ids: print("No servos found!"); return
    print(f"\nEnabling freedrive on {servo_ids}")
    print("Servos will go limp - support any load! Press Ctrl+C to stop.\n")
    driver.unload_all(servo_ids)
    try:
        while True:
            angles = driver.get_all_angles(servo_ids)
            parts = [f"J{sid}:{a:6.1f}" for sid, a in zip(servo_ids, angles)]
            print(f"\r  {'  |  '.join(parts)}", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nRe-enabling torque...")
        driver.load_all(servo_ids)
        print("Done.")


def cmd_center(driver):
    print("\n=== Scanning ===")
    servo_ids = driver.scan(range(0, 10))
    if not servo_ids: print("No servos found!"); return
    print(f"Moving {servo_ids} to center...")
    driver.move_all_to_center(servo_ids, 1500)
    time.sleep(2); print("Done!")


def cmd_interactive(driver):
    print("\n=== Interactive Mode ===")
    print("The 'driver' object is ready. Quick reference:\n")
    print("  driver.scan()                     # Find servos")
    print("  driver.get_angle(1)               # Read angle")
    print("  driver.set_angle(1, 45.0)          # Move to 45 deg")
    print("  driver.set_position(1, 500, 1000)  # Pos 500 over 1s")
    print("  driver.enable_torque(1, False)     # Disable torque")
    print("  driver.enable_torque(1, True)      # Enable torque")
    print("  driver.get_state(1)                # Full status")
    print("  driver.read_temperature(1)         # Temp in C")
    print("  driver.read_voltage(1)             # Voltage in V")
    print("  driver.set_motor_mode(1, speed=0)  # EM braking mode")
    print("  driver.set_servo_mode(1)           # Back to position mode\n")
    import code; code.interact(local={"driver": driver})


def main():
    parser = argparse.ArgumentParser(description="HX-30HM Servo Setup & Test")
    parser.add_argument("--port", "-p", type=str, default=None,
                        help="Serial port (e.g. COM3, /dev/ttyUSB0)")
    parser.add_argument("command", choices=[
        "scan", "set_id", "test", "test_one", "freedrive", "center", "interactive"])
    parser.add_argument("args", nargs="*")
    args = parser.parse_args()
    driver = find_and_connect(args.port)
    try:
        if args.command == "scan": cmd_scan(driver)
        elif args.command == "set_id":
            if len(args.args) != 2: print("Usage: set_id <current_id> <new_id>"); sys.exit(1)
            cmd_set_id(driver, int(args.args[0]), int(args.args[1]))
        elif args.command == "test": cmd_test(driver)
        elif args.command == "test_one":
            if len(args.args) != 1: print("Usage: test_one <servo_id>"); sys.exit(1)
            cmd_test(driver, [int(args.args[0])])
        elif args.command == "freedrive": cmd_freedrive(driver)
        elif args.command == "center": cmd_center(driver)
        elif args.command == "interactive": cmd_interactive(driver)
    finally:
        driver.disconnect()

if __name__ == "__main__": main()
