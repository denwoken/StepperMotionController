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
// Legacy alias to keep existing variable names in StepperMotorParameters.
typedef MotorControlReg_t MotorControlReg;

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
// Legacy alias to keep existing variable names in StepperMotorParameters.
typedef MotorStatusReg_t MotorStatusReg;

// Strongly-typed storage for all registers (not memory-mapped).
typedef struct {
    uint16_t device_id;
    uint16_t fw_version;
    uint16_t motor_count;
} DeviceRegs_t;

// Bit masks
#define MOTOR_CONTROL_EN_MASK (1u << 0)
#define MOTOR_STATUS_ENABLED_MASK (1u << 0)
#define MOTOR_STATUS_RUNNING_MASK (1u << 1)
#define MOTOR_STATUS_FAULT_MASK (1u << 2)
