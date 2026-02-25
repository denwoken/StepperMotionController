#pragma once
#include "StepperMotorControllerRegisters_structs.h"

typedef union {
	uint16_t u16w[2];
	int16_t i16w[2];
	uint8_t u8[4];
	int8_t i8[4];
	uint16_t u16;
	int16_t i16;
	uint32_t u32;
	int32_t i32;
	uint32_t raw;
} wordData;

// Snapshot helpers for consistent Modbus reads
void deviceRegsSnapshotCapture(void);
void deviceRegsSnapshotInvalidate(void);

wordData readRegisterWordData(uint8_t motor, const reg_meta_t* meta);

bool writeRegisterData(uint8_t motor, const reg_meta_t* meta, wordData value);

