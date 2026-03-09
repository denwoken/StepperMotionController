import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stepper_controller import StepperController  # noqa: E402


def main() -> None:
    # Update port name and baudrate for your setup.
    ctrl = StepperController(port="COM9", baudrate=256000, slave_id=1, timeout=0.2)

    with ctrl:
        try:
            device_id = ctrl.read_device_id()
            print(f"Device ID: 0x{device_id:04X}")

            fw = ctrl.read_fw_version()
            print("FW: map {map_version}, sw {fw_major}.{fw_minor} (raw=0x{raw:04X})".format(**fw))

            motor_count = ctrl.read_motor_count()
            print(f"Motor count: {motor_count}")


            motor = ctrl.motor(0)  # 0-based index

            motor.enable(True)
            motor.set_max_velocity(128000)
            motor.set_max_accel(64000)

            position = motor.read_current_position()
            accel = motor.read_current_accel()
            velocity = motor.read_current_velocity()
            max_vel = motor.read_max_velocity()
            max_accel = motor.read_max_accel()
            print(f"Current position: {position}")
            print(f"Current accel: {accel}")
            print(f"Current velocity: {velocity}")
            print(f"Current max velocity: {max_vel}")
            print(f"Current max acceleration: {max_accel}")

            err_code = motor.read_error_code()
            print(f"Error code: {err_code}")

            status = motor.read_status()
            position = motor.read_current_position()
            print(f"Status: {status}")
            print(f"Position: {position}")


            print("Move +2000 steps")
            motor.move_relative(64000)
            time.sleep(3)

            motor.enable(False)


        except Exception as e:
            print(e)


if __name__ == "__main__":
    main()
