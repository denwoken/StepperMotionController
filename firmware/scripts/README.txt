Scripts

Purpose:
Generate Modbus register C files from an Excel table.

Files:
- gen_modbus_regs.py — reads .xlsx and generates .h/.c.
- generateRegistersFiles.bat — runs generation with project parameters.

Run:
python gen_modbus_regs.py "..\..\DeviceRegisters.xlsx" "..\Core\DeviceRegisters" StepperMotorControllerRegisters
or
generateRegistersFiles.bat

Requirements:
- Python 3
- pandas (and an .xlsx engine such as openpyxl)

Output:
- ..\Core\DeviceRegisters\StepperMotorControllerRegisters.h
- ..\Core\DeviceRegisters\StepperMotorControllerRegisters.c




