**Overview**
This folder contains a small Python module that talks to the controller over Modbus RTU and exposes high-level methods for every register defined in `Core/DeviceRegisters`.

**Install**
1. Optional: create and activate a virtual environment.
2. Install dependencies with `python -m pip install -r requirements.txt`.

**Example**
From this folder: `python examples\basic_usage.py` (update port and baudrate).
From repo root: `python controller_sdk\python\examples\basic_usage.py`.

**Notes**
- Motor index is 0..motor_count-1 by default. Use `motor(one_based=True)` if you prefer 1..N.
- 32-bit registers use low-word then high-word order (matches firmware mapping).
- The client accepts absolute Modbus addresses (40000+) and 0-based indices.
- On `open()` the client checks register map version and raises `RegisterMapMismatchError` on mismatch. You can disable via `StepperController(..., verify_map_version=False)`.
