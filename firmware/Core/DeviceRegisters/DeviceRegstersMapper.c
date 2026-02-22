#include "DeviceRegistersMapper.h"
#include "StepperMotorController.h"

#include <limits.h>

static inline uint16_t clamp_i32_to_u16_i16(int32_t v)
{
	if (v > INT16_MAX) v = INT16_MAX;
	if (v < INT16_MIN) v = INT16_MIN;
	return (uint16_t)(int16_t)v;
}

static inline uint16_t clamp_u32_to_u16(uint32_t v)
{
	if (v > UINT16_MAX) v = UINT16_MAX;
	return (uint16_t)v;
}

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

int32_t takeRegisterData(uint8_t motor, const reg_meta_t* meta)
{
	if (!meta) return 0u;

	StepperMotorController* ctrl = getStepperMotorController();
	if (!ctrl) return 0u;

	if (meta->per_motor) {
		if (motor >= ctrl->motorCount) return 0u;
		assert_param(motor < ctrl->motorCount);
		StepperMotor* m = ctrl->motors[motor];
		if (!m) return 0u;
		const StepperMotorParameters* p = &m->parameters;

		switch (meta->id) {
		case REG_ID_CONTROL: return (int32_t)p->control.raw;
		case REG_ID_STATUS: return (int32_t)p->status.raw;
		case REG_ID_ERROR_CODE: return (int32_t)p->error_code;
		case REG_ID_MODE: return (int32_t)p->mode;

		case REG_ID_CURRENT_POS_16: return (int32_t)p->move_total_steps;
		case REG_ID_CURRENT_POS_32: return (int32_t)p->move_total_steps;

		case REG_ID_CURRENT_VELOCITY_16:
			return (int32_t)(p->current_velocity >> REG_VELOCITY16_SCALE_BITS);
		case REG_ID_CURRENT_VELOCITY_32: return (int32_t)p->current_velocity;

		case REG_ID_CURRENT_ACCEL_16:
			return (int32_t)(p->current_acceleration >> REG_ACCEL16_SCALE_BITS);
		case REG_ID_CURRENT_ACCEL_32: return (int32_t)p->current_acceleration;

		case REG_ID_TARGET_POS_16: return (int32_t)p->target_pos;
		case REG_ID_TARGET_POS_32: return (int32_t)p->target_pos;

		case REG_ID_MOVE_POS_REL_16: return 0;
		case REG_ID_MOVE_POS_REL_32: return 0;

		case REG_ID_MAX_VELOCITY_16:
			return (int32_t)(p->max_velocity >> REG_VELOCITY16_SCALE_BITS);
		case REG_ID_MAX_VELOCITY_32: return (int32_t)p->max_velocity;

		case REG_ID_MAX_ACCEL_16:
			return (int32_t)(p->max_acceleration >> REG_ACCEL16_SCALE_BITS);
		case REG_ID_MAX_ACCEL_32: return (int32_t)p->max_acceleration;
		default:
			return 0u;
		}
	}

	switch (meta->id) {
	case REG_ID_DEVICE_ID: return (int32_t)ctrl->deviceRegisters.device_id;
	case REG_ID_FW_VERSION: return (int32_t)ctrl->deviceRegisters.fw_version;
	case REG_ID_MOTOR_COUNT: return (int32_t)ctrl->deviceRegisters.motor_count;
	default:
		return 0u;
	}
}

bool writeRegisterData(uint8_t motor, const reg_meta_t* meta, int32_t value)
{
	if (!meta) return false;
	if ((meta->access & REG_ACC_W) == 0) return false;

	StepperMotorController* ctrl = getStepperMotorController();
	if (!ctrl) return false;

	if (meta->per_motor) {
		if (motor >= ctrl->motorCount) return false;
		assert_param(motor < ctrl->motorCount);
		StepperMotor* m = ctrl->motors[motor];
		if (!m) return false;
		StepperMotorParameters* p = &m->parameters;

		switch (meta->id) {
		case REG_ID_CONTROL: p->control.raw = (uint16_t)value; return true;
		case REG_ID_MODE: p->mode = (uint16_t)value; return true;

		case REG_ID_TARGET_POS_16:
			apply_target_pos(p, (int16_t)value);
			return true;
		case REG_ID_TARGET_POS_32:
			apply_target_pos(p, (int32_t)value);
			return true;

		case REG_ID_MOVE_POS_REL_16:
			apply_move_rel(p, (int16_t)value);
			return true;
		case REG_ID_MOVE_POS_REL_32:
			apply_move_rel(p, (int32_t)value);
			return true;

		case REG_ID_MAX_VELOCITY_16:
			p->max_velocity = (uint32_t)(value << REG_VELOCITY16_SCALE_BITS);
			return true;
		case REG_ID_MAX_VELOCITY_32:
			p->max_velocity = value;
			return true;

		case REG_ID_MAX_ACCEL_16:
			p->max_acceleration = (uint32_t)(value << REG_ACCEL16_SCALE_BITS);
			return true;
		case REG_ID_MAX_ACCEL_32:
			p->max_acceleration = value;
			return true;

		default:
			return false;
		}
	}

	switch (meta->id) {
	default:
		return false;
	}
}
