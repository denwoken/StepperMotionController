#pragma once
#include <stdint.h>
#include "DeviceRegisters_helpers.h"

// Bitfield views for registers (16-bit)
typedef union {
    uint16_t raw;
    struct {
        uint16_t en : 1;
        uint16_t _r01 : 1;
        uint16_t _r02 : 1;
        uint16_t _r03 : 1;
        uint16_t _r04 : 1;
        uint16_t _r05 : 1;
        uint16_t _r06 : 1;
        uint16_t _r07 : 1;
        uint16_t _r08 : 1;
        uint16_t _r09 : 1;
        uint16_t _r10 : 1; 
        uint16_t _r11 : 1;
        uint16_t _r12 : 1;
        uint16_t _r13 : 1;
        uint16_t _r14 : 1;
        uint16_t _r15 : 1;
    } bits;
} MotorControlReg_t;

typedef union {
    uint16_t raw;
    struct {
        uint16_t enabled : 1;
        uint16_t running : 1;
        uint16_t fault : 1;
        uint16_t _r03 : 1;
        uint16_t _r04 : 1;
        uint16_t _r05 : 1;
        uint16_t _r06 : 1;
        uint16_t _r07 : 1;
        uint16_t _r08 : 1;
        uint16_t _r09 : 1;
        uint16_t _r10 : 1;
        uint16_t _r11 : 1;
        uint16_t _r12 : 1;
        uint16_t _r13 : 1;
        uint16_t _r14 : 1;
        uint16_t _r15 : 1;
    } bits;
} MotorStatusReg_t;

// Strongly-typed storage for all registers (not memory-mapped).
typedef struct {
    uint16_t device_id;
    uint16_t fw_version;
    uint16_t motor_count;
} DeviceRegs_t;

/*
typedef struct {
    MotorControlReg_t control;
    MotorStatusReg_t status;
    uint16_t error_code;
    uint16_t mode;

    int16_t current_pos_16;
    int16_t current_velocity_16;
    int16_t current_accel_16;
    int16_t target_pos_16;
    int16_t move_pos_rel_16;
    uint16_t max_velocity_16;
    uint16_t max_accel_16;

    int32_t current_pos_32;
    int32_t current_velocity_32;
    int32_t current_accel_32;
    int32_t target_pos_32;
    int32_t move_pos_rel_32;
    uint32_t max_velocity_32;
    uint32_t max_accel_32;
} MotorRegs_t;
*/


// Bit masks
#define MOTOR_CONTROL_EN_MASK (1u << 0)
#define MOTOR_STATUS_ENABLED_MASK (1u << 0)
#define MOTOR_STATUS_RUNNING_MASK (1u << 1)
#define MOTOR_STATUS_FAULT_MASK (1u << 2)
