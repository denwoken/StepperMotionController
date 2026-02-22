#pragma once
#include "DeviceRegisters_structs.h"

// typedef struct {
//     DeviceRegs_t common;
//     MotorRegs_t  motor[REG_MOTOR_COUNT];

//     uint16_t snapshot[REG_ADDR_SPAN];
// } AllRegsReadSnapshot_t;



uint32_t takeRegisterData(uint8_t motor, const reg_meta_t* meta);

bool writeRegisterData(uint8_t motor, const reg_meta_t* meta, uint32_t value);
