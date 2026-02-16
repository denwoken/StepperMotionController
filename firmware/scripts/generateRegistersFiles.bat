@echo off
python gen_modbus_regs.py "../../DeviceRegisters.xlsx" "../Core/DeviceRegisters" DeviceRegisters
if errorlevel 1 (
    echo.
    echo Script failed with errorlevel %errorlevel%.
    pause
    exit /b %errorlevel%
)
exit /b 0
