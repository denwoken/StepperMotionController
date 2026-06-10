#include "DeviceRegistersMapper.h"
#include "StepperMotorController.h"

#include <limits.h>
#include <string.h>

#include "FreeRTOS.h"


typedef struct {
	DeviceRegs_t device;
	StepperMotorParameters motors[REG_MOTOR_COUNT];
	uint8_t motor_count;
	bool valid;
} device_regs_snapshot_t;

static device_regs_snapshot_t g_snapshot = {0};



void deviceRegsSnapshotCapture(void)
{
	StepperMotorController* ctrl = getStepperMotorController();
	if (!ctrl) {
		g_snapshot.valid = false;
		return;
	}

	portENTER_CRITICAL();
	g_snapshot.device = ctrl->deviceRegisters;
	g_snapshot.motor_count = ctrl->motorCount;
	if (g_snapshot.motor_count > REG_MOTOR_COUNT) g_snapshot.motor_count = REG_MOTOR_COUNT;

	for (uint8_t i = 0; i < REG_MOTOR_COUNT; i++) {
		StepperMotor* m = (i < g_snapshot.motor_count) ? ctrl->motors[i] : NULL;
		if (m) {
			memcpy(&g_snapshot.motors[i], &m->parameters, sizeof(g_snapshot.motors[i]));
		} else {
			memset(&g_snapshot.motors[i], 0, sizeof(g_snapshot.motors[i]));
		}
	}
	g_snapshot.valid = true;
	portEXIT_CRITICAL();
}

void deviceRegsSnapshotInvalidate(void)
{
	g_snapshot.valid = false;
}

wordData readRegisterWordData(uint8_t motor, const reg_meta_t* meta)
{
	wordData out = {0};
	if (!meta) return out;

	const DeviceRegs_t* device_regs = NULL;
	const StepperMotorParameters* p = NULL;

	if (g_snapshot.valid) {
		device_regs = &g_snapshot.device;
		if (meta->per_motor) {
			if (motor >= g_snapshot.motor_count) return out;
			p = &g_snapshot.motors[motor];
		}
	} else {
		StepperMotorController* ctrl = getStepperMotorController();
		if (!ctrl) return out;
		device_regs = &ctrl->deviceRegisters;
		if (meta->per_motor) {
			if (motor >= ctrl->motorCount) return out;
			StepperMotor* m = ctrl->motors[motor];
			p = &m->parameters;
		}
	}

	if (meta->per_motor && !p)
		return out;
	

	switch (meta->id) {
	case REG_ID_DEVICE_ID: out.u16 = device_regs->device_id; break;
	case REG_ID_FW_VERSION: out.u16 = device_regs->fw_version; break;
	case REG_ID_MOTOR_COUNT: out.u16 = device_regs->motor_count; break;

	case REG_ID_CONTROL: out.u16 = p->control.raw; break;
	case REG_ID_STATUS: out.u16 = p->status.raw; break;
	case REG_ID_ERROR_CODE: out.u16 = p->error_code; break;
	case REG_ID_MODE: out.u16 = p->mode; break;
#if defined(REG_ID_CMD)
	case REG_ID_CMD: out.u16 = 0u; break; // write-only
#endif

	case REG_ID_CURRENT_POS_32: out.i32 = p->move_total_steps; break;
	case REG_ID_CURRENT_VELOCITY_32: out.i32 = p->current_velocity; break;
	case REG_ID_CURRENT_ACCEL_32: out.i32 = p->current_acceleration; break;
	case REG_ID_TARGET_POS_32: out.i32 = p->target_pos; break;
	case REG_ID_MOVE_POS_REL_32: out.i32 = 0; break; // write-only
	case REG_ID_MAX_VELOCITY_32: out.u32 = p->max_velocity; break;
	case REG_ID_MAX_ACCEL_32: out.u32 = p->max_acceleration; break;
	}
	return out;
}

bool writeRegisterData(uint8_t motor, const reg_meta_t* meta, wordData value)
{
	if (!meta) return false;
	if ((meta->access & REG_ACC_W) == 0) return false;

	StepperMotorController* ctrl = getStepperMotorController();
	if (!ctrl) return false;

	if (motor >= ctrl->motorCount) return false;
	StepperMotor* m = ctrl->motors[motor];
	if (!m) return false;

	switch (meta->id) {
	case REG_ID_CONTROL: m->apply_control(m, value.u16); return true;
	case REG_ID_MODE: m->apply_mode(m, value.u16); return true;
#if defined(REG_ID_CMD)
	case REG_ID_CMD: m->apply_cmd(m, value.u16); return true;
#endif

	case REG_ID_TARGET_POS_32:
		m->apply_target_pos(m,value.i32);
		//apply_target_pos(p, value.i32);
		return true;

	case REG_ID_MOVE_POS_REL_32:
		m->apply_move_rel(m,value.i32);
		//apply_move_rel(p, value.i32);
		return true;

	case REG_ID_MAX_VELOCITY_32:
		m->parameters.max_velocity = value.u32;
		return true;

	case REG_ID_MAX_ACCEL_32:
		m->parameters.max_acceleration = value.u32;
		return true;

	default:
		return false;
	}

}
