#include "DeviceRegisters_map.h"
#include <stddef.h>

static const reg_meta_t g_reg_meta[REG_ID__COUNT] = {
    [0] = { REG_ID_DEVICE_ID, 40000, 1, 0, REG_ACC_R, REG_T_U16 },
    [1] = { REG_ID_FW_VERSION, 40001, 1, 0, REG_ACC_R, REG_T_U16 },
    [2] = { REG_ID_MOTOR_COUNT, 40002, 1, 0, REG_ACC_R, REG_T_U16 },
    [3] = { REG_ID_CONTROL, 40008, 1, 1, REG_ACC_W, REG_T_U16 },
    [4] = { REG_ID_STATUS, 40013, 1, 1, REG_ACC_R, REG_T_U16 },
    [5] = { REG_ID_ERROR_CODE, 40018, 1, 1, REG_ACC_R, REG_T_U16 },
    [6] = { REG_ID_MODE, 40023, 1, 1, REG_ACC_RW, REG_T_U16 },
    [7] = { REG_ID_CURRENT_POS_32, 40032, 2, 1, REG_ACC_R, REG_T_I32 },
    [8] = { REG_ID_CURRENT_VELOCITY_32, 40042, 2, 1, REG_ACC_R, REG_T_I32 },
    [9] = { REG_ID_CURRENT_ACCEL_32, 40052, 2, 1, REG_ACC_R, REG_T_I32 },
    [10] = { REG_ID_TARGET_POS_32, 40062, 2, 1, REG_ACC_RW, REG_T_I32 },
    [11] = { REG_ID_MOVE_POS_REL_32, 40072, 2, 1, REG_ACC_W, REG_T_I32 },
    [12] = { REG_ID_MAX_VELOCITY_32, 40082, 2, 1, REG_ACC_RW, REG_T_U32 },
    [13] = { REG_ID_MAX_ACCEL_32, 40092, 2, 1, REG_ACC_RW, REG_T_U32 },
};

static reg_lut_t g_reg_lut[REG_ADDR_SPAN];
static bool g_reg_lut_inited = false;

const reg_meta_t* get_reg_meta(reg_id_t id) {
    if (id < 0) return NULL;
    if (id >= REG_ID__COUNT) return NULL;
    const reg_meta_t* m = &g_reg_meta[id];
    return m;
}

void reg_map_init(void) {
    if(g_reg_lut_inited) return;

    // Clear LUT
    for (uint16_t i = 0; i < REG_ADDR_SPAN; ++i) {
        g_reg_lut[i].reg_id = REG_ID__RESERVED;
        g_reg_lut[i].motor = 0;
    }

    // Fill LUT from meta
    for (reg_id_t i = 0; i < REG_ID__COUNT; ++i) {
        const reg_meta_t* m = get_reg_meta(i);
        if (!m) continue;

        const uint16_t base = m->base_addr;
        const uint8_t  size = m->words;

        uint8_t motors = (m->per_motor) ? (REG_MOTOR_COUNT) : (1);
        for (uint8_t motor = 0; motor < motors; ++motor) {
            const uint16_t start = (uint16_t)(base + (uint16_t)motor * (uint16_t)size);
            for (uint8_t w = 0; w < size; ++w) {
                const uint16_t addr = (uint16_t)(start + w);
                if (addr < REG_ADDR_MIN || addr > REG_ADDR_MAX) continue;
                const uint16_t idx = (uint16_t)(addr - REG_ADDR_MIN);
                g_reg_lut[idx].reg_id = i;
                g_reg_lut[idx].motor = motor;
            }
        }
    }
    g_reg_lut_inited = true;
}

reg_lut_t reg_id_from_address(uint16_t address) {
    reg_lut_t r = { REG_ID__NONE, 0 };
    if (!g_reg_lut_inited) {
        // If not initialized, behave safely.
        return r;
    }
    if (address < REG_ADDR_MIN || address > REG_ADDR_MAX) 
        return r;
    
    const uint16_t idx = (uint16_t)(address - REG_ADDR_MIN);
    r = g_reg_lut[idx];
    return r;
}

uint16_t reg_address_from_id(reg_id_t id, uint8_t motor) {
    const reg_meta_t* m = get_reg_meta(id);
    if (!m) return 0;

    if (!m->per_motor) {
        motor = 0;
    } else if (motor >= (uint8_t)REG_MOTOR_COUNT) {
        motor = 0;
    }

    return (uint16_t)(m->base_addr + (uint16_t)motor * (uint16_t)m->words);
}

bool reg_address_check_range(uint16_t address, uint16_t count) {
    if (count == 0) return false;
    if (address < REG_ADDR_MIN) return false;
    const uint32_t end = (uint32_t)address + (uint32_t)count - 1u;
    if (end > (uint32_t)REG_ADDR_MAX) return false;
    return true;
}
