#include "DeviceRegistersMapper.h"

#include <string.h>
#include <limits.h>

#include "StepperMotorController.h"
#include "StepperMotor.h"

static StepperMotorParameters g_motor_snapshot[REGMAP_MAX_MOTORS];

static inline int32_t clamp_i32_to_i16(int32_t v)
{
    if (v > INT16_MAX) return INT16_MAX;
    if (v < INT16_MIN) return INT16_MIN;
    return v;
}

static inline uint32_t clamp_u32_to_u16(uint32_t v)
{
    if (v > UINT16_MAX) return UINT16_MAX;
    return v;
}

static StepperMotor* regmap_get_motor(const regmap_t* map, uint8_t motor)
{
    if (!map || !map->ctrl) return NULL;
    if (motor >= map->motor_count) return NULL;
    return map->ctrl->motors[motor];
}

static inline uint16_t reg_word_u32(uint32_t v, reg_word_index_t word)
{
    return (word == REG_WORD_HI) ? (uint16_t)(v >> 16) : (uint16_t)(v & 0xFFFFu);
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

void regmap_bind_controller(regmap_t* map, StepperMotorController* ctrl)
{
    if (!map) return;
    map->ctrl = ctrl;
    uint8_t count = ctrl ? ctrl->motorCount : 0u;
    if (count > REGMAP_MAX_MOTORS) count = REGMAP_MAX_MOTORS;
    map->motor_count = count;
}

void regmap_capture(const regmap_t* map)
{
    if (!map) return;
    for (uint8_t i = 0u; i < map->motor_count && i < REGMAP_MAX_MOTORS; i++) {
        StepperMotor* motor = regmap_get_motor(map, i);
        if (motor) {
            memcpy(&g_motor_snapshot[i], &motor->parameters, sizeof(g_motor_snapshot[i]));
        } else {
            memset(&g_motor_snapshot[i], 0, sizeof(g_motor_snapshot[i]));
        }
    }
}

reg_word_index_t regmap_word_index(uint16_t address, uint8_t motor, reg_id_t id)
{
    const reg_meta_t* meta = get_reg_meta_by_id(id);
    uint32_t addr = address;
    if (addr < REG_BASE_ADDRESS) addr += REG_BASE_ADDRESS;

    uint32_t base = 0u;
    if (meta->motorReg) {
        base = reg_address(motor, id);
    } else {
        base = (uint32_t)REG_BASE_ADDRESS + (uint32_t)meta->index;
    }
    uint32_t diff = addr - base;
    return (diff & 1u) ? REG_WORD_HI : REG_WORD_LO;
}

bool regmap_req_covers_u32(uint16_t req_index, uint16_t req_count, uint8_t motor, reg_id_t id)
{
    if (req_count < 2u) return false;
    const reg_meta_t* meta = get_reg_meta_by_id(id);
    uint32_t start = req_index;
    if (start < REG_BASE_ADDRESS) start += REG_BASE_ADDRESS;
    uint32_t end = start + (uint32_t)req_count - 1u;

    uint32_t reg_start = 0u;
    if (meta->motorReg) {
        reg_start = reg_address(motor, id);
    } else {
        reg_start = (uint32_t)REG_BASE_ADDRESS + (uint32_t)meta->index;
    }

    return (start <= reg_start) && (end >= (reg_start + 1u));
}

bool regmap_get_access(uint8_t motor, reg_id_t id, reg_access_t* out_access)
{
    const reg_meta_t* meta = get_reg_meta_by_id(id);
    if (!meta->motorReg) return false;
    if (motor >= REGMAP_MAX_MOTORS) return false;
    if (out_access) *out_access = meta->access;
    return true;
}

uint16_t regmap_read_snapshot(const regmap_t* map, uint8_t motor, reg_id_t id, reg_word_index_t word)
{
    if (!map || motor >= map->motor_count || motor >= REGMAP_MAX_MOTORS) return 0u;
    const StepperMotorParameters* p = &g_motor_snapshot[motor];

    switch (id) {
    case REG_CONTROL: return p->control.raw;
    case REG_STATUS: return p->status.raw;
    case REG_ERROR_CODE: return p->error_code;
    case REG_MODE: return p->mode;
    case REG_CMD: return p->cmd;

    case REG_CURRENT_POS_16: return (uint16_t)clamp_i32_to_i16(p->move_total_steps);
    case REG_CURRENT_POS_32: return reg_word_u32((uint32_t)p->move_total_steps, word);

    case REG_CURRENT_VELOCITY_16: return (uint16_t)clamp_i32_to_i16(p->current_velocity / 8);
    case REG_CURRENT_VELOCITY_32: return reg_word_u32((uint32_t)p->current_velocity, word);

    case REG_CURRENT_ACCEL_16: return (uint16_t)clamp_i32_to_i16(p->current_acceleration / 256);
    case REG_CURRENT_ACCEL_32: return reg_word_u32((uint32_t)p->current_acceleration, word);

    case REG_TARGET_POS_16: return (uint16_t)clamp_i32_to_i16(p->target_pos);
    case REG_TARGET_POS_32: return reg_word_u32((uint32_t)p->target_pos, word);

    case REG_MAX_VELOCITY_16: return (uint16_t)clamp_u32_to_u16(p->max_velocity / 8u);
    case REG_MAX_VELOCITY_32: return reg_word_u32(p->max_velocity, word);

    case REG_MAX_ACCEL_16: return (uint16_t)clamp_u32_to_u16(p->max_acceleration / 256u);
    case REG_MAX_ACCEL_32: return reg_word_u32(p->max_acceleration, word);

    case REG_MOVE_POS_REL_16: return (uint16_t)clamp_i32_to_i16(p->move_pos_rel);
    case REG_MOVE_POS_REL_32: return reg_word_u32((uint32_t)p->move_pos_rel, word);

    default:
        return 0u;
    }
}

bool regmap_write_u16_direct(const regmap_t* map, uint8_t motor, reg_id_t id, uint16_t value)
{
    const reg_meta_t* meta = get_reg_meta_by_id(id);
    if (!meta->motorReg || meta->size != 2u) return false;

    StepperMotor* m = regmap_get_motor(map, motor);
    if (!m) return false;
    StepperMotorParameters* p = &m->parameters;

    switch (id) {
    case REG_CONTROL: p->control.raw = value; return true;
    case REG_MODE: p->mode = value; return true;
    case REG_CMD: p->cmd = value; return true;

    case REG_TARGET_POS_16:
        apply_target_pos(p, (int32_t)(int16_t)value);
        return true;
    case REG_MAX_VELOCITY_16:
        p->max_velocity = (uint32_t)value * 8u;
        return true;
    case REG_MAX_ACCEL_16:
        p->max_acceleration = (uint32_t)value * 256u;
        return true;
    case REG_MOVE_POS_REL_16:
        apply_move_rel(p, (int32_t)(int16_t)value);
        return true;
    default:
        return false;
    }
}

bool regmap_write_u32_direct(const regmap_t* map, uint8_t motor, reg_id_t id, uint32_t value)
{
    const reg_meta_t* meta = get_reg_meta_by_id(id);
    if (!meta->motorReg || meta->size != 4u) return false;

    StepperMotor* m = regmap_get_motor(map, motor);
    if (!m) return false;
    StepperMotorParameters* p = &m->parameters;

    switch (id) {
    case REG_TARGET_POS_32:
        apply_target_pos(p, (int32_t)value);
        return true;
    case REG_MAX_VELOCITY_32:
        p->max_velocity = value;
        return true;
    case REG_MAX_ACCEL_32:
        p->max_acceleration = value;
        return true;
    case REG_MOVE_POS_REL_32:
        apply_move_rel(p, (int32_t)value);
        return true;
    default:
        return false;
    }
}
