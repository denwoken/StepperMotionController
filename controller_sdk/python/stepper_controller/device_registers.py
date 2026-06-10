from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Dict

# Derived from Core/DeviceRegisters/StepperMotorControllerRegisters_map.h/.c
REG_BASE_ADDRESS = 40000
REG_COMMON_START = 0
REG_MOTOR_CONTROL_OFFSET = 8
REG_MOTOR32_OFFSET = 32
REG_MOTOR_COUNT = 5
REG_ADDR_MIN = 40000
REG_ADDR_MAX = 40101
REG_ADDR_SPAN = 102

REG_MAP_VERSION = 1


class RegAccess(Enum):
    R = "R"
    W = "W"
    RW = "RW"


class RegType(Enum):
    U16 = "U16"
    I16 = "I16"
    U32 = "U32"
    I32 = "I32"


class RegId(IntEnum):
    DEVICE_ID = 0
    FW_VERSION = 1
    MOTOR_COUNT = 2
    CONTROL = 3
    STATUS = 4
    ERROR_CODE = 5
    MODE = 6
    CURRENT_POS_32 = 7
    CURRENT_VELOCITY_32 = 8
    CURRENT_ACCEL_32 = 9
    TARGET_POS_32 = 10
    MOVE_POS_REL_32 = 11
    MAX_VELOCITY_32 = 12
    MAX_ACCEL_32 = 13


@dataclass(frozen=True)
class RegMeta:
    id: RegId
    base_addr: int
    words: int
    per_motor: bool
    access: RegAccess
    dtype: RegType


REG_BASE_DEVICE_ID = 40000
REG_SIZE_DEVICE_ID = 1
REG_PER_MOTOR_DEVICE_ID = 0

REG_BASE_FW_VERSION = 40001
REG_SIZE_FW_VERSION = 1
REG_PER_MOTOR_FW_VERSION = 0

REG_BASE_MOTOR_COUNT = 40002
REG_SIZE_MOTOR_COUNT = 1
REG_PER_MOTOR_MOTOR_COUNT = 0

REG_BASE_CONTROL = 40008
REG_SIZE_CONTROL = 1
REG_PER_MOTOR_CONTROL = 1

REG_BASE_STATUS = 40013
REG_SIZE_STATUS = 1
REG_PER_MOTOR_STATUS = 1

REG_BASE_ERROR_CODE = 40018
REG_SIZE_ERROR_CODE = 1
REG_PER_MOTOR_ERROR_CODE = 1

REG_BASE_MODE = 40023
REG_SIZE_MODE = 1
REG_PER_MOTOR_MODE = 1

REG_BASE_CURRENT_POS_32 = 40032
REG_SIZE_CURRENT_POS_32 = 2
REG_PER_MOTOR_CURRENT_POS_32 = 1

REG_BASE_CURRENT_VELOCITY_32 = 40042
REG_SIZE_CURRENT_VELOCITY_32 = 2
REG_PER_MOTOR_CURRENT_VELOCITY_32 = 1

REG_BASE_CURRENT_ACCEL_32 = 40052
REG_SIZE_CURRENT_ACCEL_32 = 2
REG_PER_MOTOR_CURRENT_ACCEL_32 = 1

REG_BASE_TARGET_POS_32 = 40062
REG_SIZE_TARGET_POS_32 = 2
REG_PER_MOTOR_TARGET_POS_32 = 1

REG_BASE_MOVE_POS_REL_32 = 40072
REG_SIZE_MOVE_POS_REL_32 = 2
REG_PER_MOTOR_MOVE_POS_REL_32 = 1

REG_BASE_MAX_VELOCITY_32 = 40082
REG_SIZE_MAX_VELOCITY_32 = 2
REG_PER_MOTOR_MAX_VELOCITY_32 = 1

REG_BASE_MAX_ACCEL_32 = 40092
REG_SIZE_MAX_ACCEL_32 = 2
REG_PER_MOTOR_MAX_ACCEL_32 = 1


REG_META: Dict[RegId, RegMeta] = {
    RegId.DEVICE_ID: RegMeta(RegId.DEVICE_ID, 40000, 1, False, RegAccess.R, RegType.U16),
    RegId.FW_VERSION: RegMeta(RegId.FW_VERSION, 40001, 1, False, RegAccess.R, RegType.U16),
    RegId.MOTOR_COUNT: RegMeta(RegId.MOTOR_COUNT, 40002, 1, False, RegAccess.R, RegType.U16),
    
    RegId.CONTROL: RegMeta(RegId.CONTROL, 40008, 1, True, RegAccess.W, RegType.U16),
    RegId.STATUS: RegMeta(RegId.STATUS, 40013, 1, True, RegAccess.R, RegType.U16),
    RegId.ERROR_CODE: RegMeta(RegId.ERROR_CODE, 40018, 1, True, RegAccess.R, RegType.U16),
    RegId.MODE: RegMeta(RegId.MODE, 40023, 1, True, RegAccess.RW, RegType.U16),
    RegId.CURRENT_POS_32: RegMeta(RegId.CURRENT_POS_32, 40032, 2, True, RegAccess.R, RegType.I32),
    RegId.CURRENT_VELOCITY_32: RegMeta(RegId.CURRENT_VELOCITY_32, 40042, 2, True, RegAccess.R, RegType.I32),
    RegId.CURRENT_ACCEL_32: RegMeta(RegId.CURRENT_ACCEL_32, 40052, 2, True, RegAccess.R, RegType.I32),
    RegId.TARGET_POS_32: RegMeta(RegId.TARGET_POS_32, 40062, 2, True, RegAccess.RW, RegType.I32),
    RegId.MOVE_POS_REL_32: RegMeta(RegId.MOVE_POS_REL_32, 40072, 2, True, RegAccess.W, RegType.I32),
    RegId.MAX_VELOCITY_32: RegMeta(RegId.MAX_VELOCITY_32, 40082, 2, True, RegAccess.RW, RegType.U32),
    RegId.MAX_ACCEL_32: RegMeta(RegId.MAX_ACCEL_32, 40092, 2, True, RegAccess.RW, RegType.U32),
}


# Bit masks from StepperMotorControllerRegisters_structs.h
MOTOR_CONTROL_EN_MASK = 0x0001
MOTOR_STATUS_ENABLED_MASK = 0x0001
MOTOR_STATUS_RUNNING_MASK = 0x0002
MOTOR_STATUS_FAULT_MASK = 0x0004


def get_reg_meta(reg_id: RegId) -> RegMeta:
    return REG_META[reg_id]


def reg_address(reg_id: RegId, motor: int = 0) -> int:
    meta = get_reg_meta(reg_id)
    if meta.per_motor:
        if motor < 0 or motor >= REG_MOTOR_COUNT:
            raise ValueError(f"motor out of range: {motor}")
        return meta.base_addr + motor * meta.words
    return meta.base_addr


def fw_version_unpack(u16: int) -> Dict[str, int]:
    map_ver = (u16 >> 8) & 0xFF
    fw_sw = u16 & 0xFF
    return {
        "raw": u16 & 0xFFFF,
        "map_version": map_ver,
        "fw_sw": fw_sw,
        "fw_major": fw_sw // 10,
        "fw_minor": fw_sw % 10,
    }
