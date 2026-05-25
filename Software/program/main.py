"""ArmCTL — Entry point.

  python main.py                  # real hardware on COM5 (default)
  python main.py --port COM3      # real hardware on a different port
  python main.py --sim            # simulated driver, no hardware needed
  python main.py --sim --no-gui   # scripting mode with sim
"""
import sys, argparse
from armctl.config.arm_config import DEFAULT_CONFIG
from armctl.core.arm import ArmController
from armctl.drivers.simulated import SimulatedDriver

def main():
    parser = argparse.ArgumentParser(description="ArmCTL Robot Arm Controller")
    parser.add_argument("--port",   type=str, default="COM5",       help="Serial port (default: COM5)")
    parser.add_argument("--sim",    action="store_true",             help="Use simulated driver instead of real hardware")
    parser.add_argument("--no-gui", action="store_true",             help="Scripting mode (no GUI)")
    args = parser.parse_args()

    if args.sim:
        driver = SimulatedDriver()
        port = None
    else:
        from armctl.drivers.hiwonder import HiwonderDriver
        driver = HiwonderDriver()
        port = args.port

    arm = ArmController(driver, DEFAULT_CONFIG)
    if not arm.connect(port):
        print("Failed to connect!"); sys.exit(1)
    arm.home()

    if args.no_gui:
        print("ArmCTL scripting mode. Use 'arm' to control the robot.")
        import code; code.interact(local={"arm": arm, "gripper": arm.gripper, "config": DEFAULT_CONFIG})
    else:
        try:
            from armctl.gui.app import run_gui; run_gui(arm)
        except ImportError:
            print("tkinter not available. Falling back to scripting mode.")
            import code; code.interact(local={"arm": arm, "gripper": arm.gripper, "config": DEFAULT_CONFIG})
    arm.disconnect()

if __name__ == "__main__": main()
