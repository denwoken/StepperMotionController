# StepperMotionController
StepperMotionController is a full hardware + firmware + software stack for controlling multiple stepper motors over RS-485 (Modbus RTU). You configure the required motion parameters, set a target position, and the controller computes and executes the motion trajectory and speed profile based on current speed and acceleration limits.

This repo contains the STM32G0 firmware, the PCB design, and a Python SDK.

Planned: feedback sensor support for detecting missed steps, plus richer driver control via a shared UART connected to all TMC2209 drivers.

**Repository Layout**
- `firmware/` STM32G071 firmware (FreeRTOS) that exposes a Modbus RTU register map for motor control.
- `controller_sdk/python/` Python SDK built on `pyserial` with a high-level API and examples.
- `Altium/` PCB/schematic sources for the controller hardware.
- `DeviceRegisters.xlsx` Source-of-truth register map.
- `mbpool device registers.mbw` Modbus Poll workspace for register testing.
- `firmware/Core/DeviceRegisters/` Generated C register map used by firmware.
- `firmware/scripts/` Tools to generate register map files from the Excel table.
- `LegacyDeviceRegisters/` Older register map versions and outputs.
- `firmware/Doc/` MCU datasheet, reference manual, and FreeRTOS docs.
- `firmware.code-workspace` VS Code workspace for firmware development.
- `AltiumProjectGroup.DsnWrk` Altium project group workspace file.

**Quick Start (Python)**
1. Install dependencies:
   `python -m pip install -r controller_sdk/python/requirements.txt`
2. Run the example (edit port and baudrate as needed):
   `python controller_sdk/python/examples/basic_usage.py`

Minimal python usage:
```python
from stepper_controller import StepperController

with StepperController(port="COM9", baudrate=256000, slave_id=1) as ctrl:
    print(ctrl.read_device_id())
    motor = ctrl.motor(0)
    motor.enable(True)
    motor.set_max_velocity(128000)
    motor.move_relative(64000)
```

**Firmware Build**
1. Open `firmware.code-workspace` in VScode press F5 to build and start debugging session.


Notes:
- Modbus RTU is configured on `USART1` with RS-485 DE; the default baudrate is `256000`.
- The firmware expects the register map generated into `firmware/Core/DeviceRegisters/`.

**Register Map**
The register map starts in `DeviceRegisters.xlsx` and is used by both firmware and the Python SDK.



SDK compatibility:
- `controller_sdk/python/stepper_controller/device_registers.py` mirrors the firmware map.
- The SDK checks the map version on `open()` and raises `RegisterMapMismatchError` if it differs. You can disable this check with `StepperController(..., verify_map_version=False)`.

**Hardware**
Open the Altium project in `Altium/` to view schematics and PCB layout.

**License**
MIT. See `LICENSE`.
