from __future__ import annotations

from typing import Dict, List

from .device_registers import (
    MOTOR_CONTROL_EN_MASK,
    MOTOR_STATUS_ENABLED_MASK,
    MOTOR_STATUS_FAULT_MASK,
    MOTOR_STATUS_RUNNING_MASK,
    REG_MAP_VERSION,
    REG_MOTOR_COUNT,
    RegAccess,
    RegId,
    RegMeta,
    RegType,
    fw_version_unpack,
    get_reg_meta,
    reg_address,
)
from .modbus_rtu import ModbusRTUClient


class RegisterMapMismatchError(RuntimeError):
    pass


class StepperController:
    def __init__(
        self,
        port: str,
        baudrate: int = 256000,
        slave_id: int = 1,
        timeout: float = 0.2,
        parity: str = "N",
        stopbits: int = 1,
        bytesize: int = 8,
        verify_map_version: bool = True,
        expected_map_version: int = REG_MAP_VERSION,
    ) -> None:
        self._client = ModbusRTUClient(
            port=port,
            baudrate=baudrate,
            slave_id=slave_id,
            timeout=timeout,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
        )
        self._verify_map_version = bool(verify_map_version)
        self._expected_map_version = int(expected_map_version)

    def open(self) -> None:
        self._client.open()
        self._check_register_map_version()

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "StepperController":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def motor(self, index: int, one_based: bool = False) -> "Motor":
        if one_based:
            index -= 1
        return Motor(self, index)

    def read_device_id(self) -> int:
        return int(self.read_register(RegId.DEVICE_ID))

    def read_fw_version(self) -> Dict[str, int]:
        raw = int(self.read_register(RegId.FW_VERSION))
        return fw_version_unpack(raw)

    def read_motor_count(self) -> int:
        return int(self.read_register(RegId.MOTOR_COUNT))

    def read_register(self, reg_id: RegId, motor: int = 0) -> int:
        meta = get_reg_meta(reg_id)
        if meta.access == RegAccess.W:
            raise ValueError(f"register {reg_id.name} is write-only")
        addr = reg_address(reg_id, motor)
        words = self._client.read_holding_registers(addr, meta.words)
        return _decode_value(meta, words)

    def write_register(self, reg_id: RegId, value: int, motor: int = 0) -> None:
        meta = get_reg_meta(reg_id)
        if meta.access == RegAccess.R:
            raise ValueError(f"register {reg_id.name} is read-only")
        addr = reg_address(reg_id, motor)
        words = _encode_value(meta, value)
        if meta.words == 1:
            self._client.write_single_register(addr, words[0])
        else:
            self._client.write_multiple_registers(addr, words)

    def _check_register_map_version(self) -> None:
        if not self._verify_map_version:
            return
        info = self.read_fw_version()
        actual = int(info["map_version"])
        expected = self._expected_map_version
        if actual != expected:
            self.close()
            raise RegisterMapMismatchError(
                "register map version mismatch: "
                f"expected {expected}, device {actual} (fw_raw=0x{info['raw']:04X})"
            )


class Motor:
    def __init__(self, controller: StepperController, index: int) -> None:
        if index < 0 or index >= REG_MOTOR_COUNT:
            raise ValueError(f"motor index out of range: {index}")
        self._c = controller
        self.index = index

    def enable(self, enabled: bool = True) -> None:
        value = MOTOR_CONTROL_EN_MASK if enabled else 0
        self._c.write_register(RegId.CONTROL, value, motor=self.index)

    def write_control_raw(self, value: int) -> None:
        self._c.write_register(RegId.CONTROL, value, motor=self.index)

    def read_status_raw(self) -> int:
        return int(self._c.read_register(RegId.STATUS, motor=self.index))

    def read_status(self) -> Dict[str, bool]:
        raw = self.read_status_raw()
        return {
            "enabled": bool(raw & MOTOR_STATUS_ENABLED_MASK),
            "running": bool(raw & MOTOR_STATUS_RUNNING_MASK),
            "fault": bool(raw & MOTOR_STATUS_FAULT_MASK),
        }

    def read_error_code(self) -> int:
        return int(self._c.read_register(RegId.ERROR_CODE, motor=self.index))

    #def read_mode(self) -> int:
    #    return int(self._c.read_register(RegId.MODE, motor=self.index))

    #def set_mode(self, mode: int) -> None:
    #    self._c.write_register(RegId.MODE, int(mode), motor=self.index)

    def read_current_position(self) -> int:
        return int(self._c.read_register(RegId.CURRENT_POS_32, motor=self.index))

    def read_current_velocity(self) -> int:
        return int(self._c.read_register(RegId.CURRENT_VELOCITY_32, motor=self.index))

    def read_current_accel(self) -> int:
        return int(self._c.read_register(RegId.CURRENT_ACCEL_32, motor=self.index))


    def read_target_position(self) -> int:
        return int(self._c.read_register(RegId.TARGET_POS_32, motor=self.index))

    def set_target_position(self, value: int) -> None:
        self._c.write_register(RegId.TARGET_POS_32, int(value), motor=self.index)


    def move_relative(self, value: int) -> None:
        self._c.write_register(RegId.MOVE_POS_REL_32, int(value), motor=self.index)


    def read_max_velocity(self) -> int:
        return int(self._c.read_register(RegId.MAX_VELOCITY_32, motor=self.index))

    def set_max_velocity(self, value: int) -> None:
        self._c.write_register(RegId.MAX_VELOCITY_32, int(value), motor=self.index)

    def read_max_accel(self) -> int:
        return int(self._c.read_register(RegId.MAX_ACCEL_32, motor=self.index))

    def set_max_accel(self, value: int) -> None:
        self._c.write_register(RegId.MAX_ACCEL_32, int(value), motor=self.index)


def _encode_value(meta: RegMeta, value: int) -> List[int]:
    if meta.dtype == RegType.U16:
        if not (0 <= value <= 0xFFFF):
            raise ValueError("u16 out of range")
        return [value]
    if meta.dtype == RegType.I16:
        if not (-0x8000 <= value <= 0x7FFF):
            raise ValueError("i16 out of range")
        if value < 0:
            value = (1 << 16) + value
        return [value & 0xFFFF]
    if meta.dtype == RegType.U32:
        if not (0 <= value <= 0xFFFFFFFF):
            raise ValueError("u32 out of range")
        return _split_u32(value)
    if meta.dtype == RegType.I32:
        if not (-0x80000000 <= value <= 0x7FFFFFFF):
            raise ValueError("i32 out of range")
        return _split_i32(value)
    raise ValueError("unknown register type")


def _decode_value(meta: RegMeta, words: List[int]) -> int:
    if meta.dtype == RegType.U16:
        return int(words[0])
    if meta.dtype == RegType.I16:
        v = int(words[0])
        if v & 0x8000:
            v -= 0x10000
        return v
    if meta.dtype == RegType.U32:
        return _join_u32(words)
    if meta.dtype == RegType.I32:
        return _join_i32(words)
    raise ValueError("unknown register type")


def _split_u32(value: int) -> List[int]:
    low = value & 0xFFFF
    high = (value >> 16) & 0xFFFF
    return [low, high]


def _split_i32(value: int) -> List[int]:
    if value < 0:
        value = (1 << 32) + value
    return _split_u32(value)


def _join_u32(words: List[int]) -> int:
    if len(words) != 2:
        raise ValueError("u32 requires 2 words")
    low, high = words[0], words[1]
    return (high << 16) | low


def _join_i32(words: List[int]) -> int:
    u = _join_u32(words)
    if u & 0x80000000:
        return u - 0x100000000
    return u
