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


static inline int16_t clamp_i32_to_i16(int32_t v)
{
	if (v > INT16_MAX) v = INT16_MAX;
	if (v < INT16_MIN) v = INT16_MIN;
	return (int16_t)v;
}

static inline uint16_t clamp_u32_to_u16(uint32_t v)
{
	if (v > UINT16_MAX) v = UINT16_MAX;
	return (uint16_t)v;
}

static inline wordData pack_word_data(const reg_meta_t* meta, int32_t s, uint32_t u)
{
	wordData out = {0};
	if (!meta) return out;

	// wordData.raw stores register bits, dtype defines interpretation.
	switch (meta->dtype) {
	case REG_T_I16: out.raw = (uint32_t)(uint16_t)clamp_i32_to_i16(s); break;
	case REG_T_U16: out.raw = (uint32_t)clamp_u32_to_u16(u); break;
	case REG_T_I32: out.raw = (uint32_t)s; break;
	case REG_T_U32: out.raw = u; break;
	default: out.raw = 0u; break;
	}
	return out;
}

static void apply_control(StepperMotorParameters* p, uint16_t value)
{
	// TODO: implement full control behavior (enable/disable, etc.)
	p->control.raw = value;

	// Simple sketch: mirror enable bit into status and stop on disable.
	if ((value & MOTOR_CONTROL_EN_MASK) == 0u) {
		// p->remaining_steps = 0;
		// p->current_velocity = 0;
		// p->current_acceleration = 0;
		// p->move_pos_rel = 0;
		// p->target_pos = p->move_total_steps;
		// p->status.bits.running = 0u;
	}
	//p->status.bits.enabled = (value & MOTOR_CONTROL_EN_MASK) ? 1u : 0u;
}

static void apply_mode(StepperMotorParameters* p, uint16_t value)
{
	// TODO: define mode semantics (position/velocity/step, etc.)
	p->mode = value;
}


#if defined(REG_ID_CMD)
static void apply_cmd(StepperMotorParameters* p, uint16_t value)
{
	// Write-only command register sketch.
	// TODO: replace with real command decoding.
	p->cmd = value;
	if (value & 0x0001u) {
		// CMD bit0: stop motion (placeholder action)
		p->remaining_steps = 0;
		p->current_velocity = 0;
		p->current_acceleration = 0;
		p->move_pos_rel = 0;
		p->target_pos = p->move_total_steps;
		p->status.bits.running = 0u;
	}
}
#endif

static void apply_target_pos(StepperMotorParameters* p, int32_t target)
{
	p->target_pos = target;
	p->remaining_steps = p->target_pos - p->move_total_steps;
	p->move_pos_rel = 0;
}

static void apply_move_rel(StepperMotorParameters* p, int32_t move)
{
	p->remaining_steps += move;
	p->target_pos = p->move_total_steps + p->remaining_steps;
	p->move_pos_rel = 0;
}

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

	if (meta->per_motor && !p) {
		return out;
	}

	int32_t s = 0;
	uint32_t u = 0;

	switch (meta->id) {
	case REG_ID_DEVICE_ID: u = device_regs->device_id; s = (int32_t)u; break;
	case REG_ID_FW_VERSION: u = device_regs->fw_version; s = (int32_t)u; break;
	case REG_ID_MOTOR_COUNT: u = device_regs->motor_count; s = (int32_t)u; break;

	case REG_ID_CONTROL: u = p->control.raw; s = (int32_t)u; break;
	case REG_ID_STATUS: u = p->status.raw; s = (int32_t)u; break;
	case REG_ID_ERROR_CODE: u = p->error_code; s = (int32_t)u; break;
	case REG_ID_MODE: u = p->mode; s = (int32_t)u; break;
#if defined(REG_ID_CMD)
	case REG_ID_CMD: u = 0u; s = 0; break; // write-only
#endif

	case REG_ID_CURRENT_POS_16: s = (int32_t)clamp_i32_to_i16(p->move_total_steps); u = (uint16_t)s; break;
	case REG_ID_CURRENT_POS_32: s = p->move_total_steps; u = (uint32_t)s; break;

	case REG_ID_CURRENT_VELOCITY_16:
		s = (int32_t)clamp_i32_to_i16(p->current_velocity >> REG_VELOCITY16_SCALE_BITS);
		u = (uint16_t)s;
		break;
	case REG_ID_CURRENT_VELOCITY_32: s = p->current_velocity; u = (uint32_t)s; break;

	case REG_ID_CURRENT_ACCEL_16:
		s = (int32_t)clamp_i32_to_i16(p->current_acceleration >> REG_ACCEL16_SCALE_BITS);
		u = (uint16_t)s;
		break;
	case REG_ID_CURRENT_ACCEL_32: s = p->current_acceleration; u = (uint32_t)s; break;

	case REG_ID_TARGET_POS_16: s = (int32_t)clamp_i32_to_i16(p->target_pos); u = (uint16_t)s; break;
	case REG_ID_TARGET_POS_32: s = p->target_pos; u = (uint32_t)s; break;

	case REG_ID_MOVE_POS_REL_16: s = 0; u = 0u; break;
	case REG_ID_MOVE_POS_REL_32: s = 0; u = 0u; break;

	case REG_ID_MAX_VELOCITY_16:
		u = clamp_u32_to_u16(p->max_velocity >> REG_VELOCITY16_SCALE_BITS);
		s = (int32_t)u;
		break;
	case REG_ID_MAX_VELOCITY_32: u = p->max_velocity; s = (int32_t)u; break;

	case REG_ID_MAX_ACCEL_16:
		u = clamp_u32_to_u16(p->max_acceleration >> REG_ACCEL16_SCALE_BITS);
		s = (int32_t)u;
		break;
	case REG_ID_MAX_ACCEL_32: u = p->max_acceleration; s = (int32_t)u; break;
	default:
		s = 0; u = 0u; break;
	}

	return pack_word_data(meta, s, u);
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
	StepperMotorParameters* p = &m->parameters;

	switch (meta->id) {
	case REG_ID_CONTROL: apply_control(p, value.u16); return true;
	case REG_ID_MODE: apply_mode(p, value.u16); return true;
#if defined(REG_ID_CMD)
	case REG_ID_CMD: apply_cmd(p, value.u16); return true;
#endif

	case REG_ID_TARGET_POS_16:
		apply_target_pos(p, value.i16);
		return true;
	case REG_ID_TARGET_POS_32:
		apply_target_pos(p, value.i32);
		return true;

	case REG_ID_MOVE_POS_REL_16:
		apply_move_rel(p, value.i16);
		return true;
	case REG_ID_MOVE_POS_REL_32:
		apply_move_rel(p, value.i32);
		return true;

	case REG_ID_MAX_VELOCITY_16:
		p->max_velocity = (uint32_t)(value.u16 << REG_VELOCITY16_SCALE_BITS);
		return true;
	case REG_ID_MAX_VELOCITY_32:
		p->max_velocity = value.u32;
		return true;

	case REG_ID_MAX_ACCEL_16:
		p->max_acceleration = (uint32_t)(value.u16 << REG_ACCEL16_SCALE_BITS);
		return true;
	case REG_ID_MAX_ACCEL_32:
		p->max_acceleration = value.u32;
		return true;

	default:
		return false;
	}

}
