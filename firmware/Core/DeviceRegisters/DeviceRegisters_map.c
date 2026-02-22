#include "DeviceRegisters_map.h"



static const reg_meta_t g_reg_meta[REG_ID__COUNT] = {
    /* DEVICE */ { REG_ID_DEVICE_ID, 40000, 1, 0, REG_ACC_R, REG_T_U16 },
    /* DEVICE */ { REG_ID_FW_VERSION, 40001, 1, 0, REG_ACC_R, REG_T_U16 },
    /* DEVICE */ { REG_ID_MOTOR_COUNT, 40002, 1, 0, REG_ACC_R, REG_T_U16 },

    /* MOTOR_CTRL */ { REG_ID_CONTROL, 40008, 1, 1, REG_ACC_W, REG_T_U16 },
    /* MOTOR_CTRL */ { REG_ID_STATUS, 40013, 1, 1, REG_ACC_R, REG_T_U16 },
    /* MOTOR_CTRL */ { REG_ID_ERROR_CODE, 40018, 1, 1, REG_ACC_R, REG_T_U16 },
    /* MOTOR_CTRL */ { REG_ID_MODE, 40023, 1, 1, REG_ACC_RW, REG_T_U16 },

    /* MOTION16 */ { REG_ID_CURRENT_POS_16, 40032, 1, 1, REG_ACC_R, REG_T_I16 },
    /* MOTION16 */ { REG_ID_CURRENT_VELOCITY_16, 40037, 1, 1, REG_ACC_R, REG_T_I16 },
    /* MOTION16 */ { REG_ID_CURRENT_ACCEL_16, 40042, 1, 1, REG_ACC_R, REG_T_I16 },
    /* MOTION16 */ { REG_ID_TARGET_POS_16, 40047, 1, 1, REG_ACC_RW, REG_T_I16  },
    /* MOTION16 */ { REG_ID_MOVE_POS_REL_16, 40052, 1, 1, REG_ACC_W, REG_T_I16  },
    /* MOTION16 */ { REG_ID_MAX_VELOCITY_16, 40057, 1, 1, REG_ACC_RW, REG_T_U16  },
    /* MOTION16 */ { REG_ID_MAX_ACCEL_16, 40062, 1, 1, REG_ACC_RW, REG_T_U16  },

    /* MOTION32 */ { REG_ID_CURRENT_POS_32, 40068, 2, 1, REG_ACC_R, REG_T_I32 },
    /* MOTION32 */ { REG_ID_CURRENT_VELOCITY_32, 40078, 2, 1, REG_ACC_R, REG_T_I32 },
    /* MOTION32 */ { REG_ID_CURRENT_ACCEL_32, 40088, 2, 1, REG_ACC_R, REG_T_I32 },
    /* MOTION32 */ { REG_ID_TARGET_POS_32, 40098, 2, 1, REG_ACC_RW, REG_T_I32 },
    /* MOTION32 */ { REG_ID_MOVE_POS_REL_32, 40108, 2, 1, REG_ACC_W, REG_T_I32 },
    /* MOTION32 */ { REG_ID_MAX_VELOCITY_32, 40118, 2, 1, REG_ACC_RW, REG_T_U32 },
    /* MOTION32 */ { REG_ID_MAX_ACCEL_32, 40128, 2, 1, REG_ACC_RW, REG_T_U32 },
};




static reg_lut_t g_reg_lut[REG_ADDR_SPAN];
static bool g_reg_lut_inited = false;

const reg_meta_t* get_reg_meta(reg_id_t id) {
    if (id < 0) return NULL;
    if (id >= REG_ID__COUNT) return NULL;
    return &g_reg_meta[id];
}

void reg_map_init(void) {
    if(g_reg_lut_inited) return;

    // Clear LUT
    for (uint16_t i = 0; i < REG_ADDR_SPAN; ++i) {
        g_reg_lut[i].reg_id = REG_ID__RESERVED;
        g_reg_lut[i].motor = 0;
    }

    // Fill LUT from meta
    for (reg_id_t i = (reg_id_t)0; i < (uint32_t)REG_ID__COUNT; ++i) {
        const reg_meta_t* m = get_reg_meta(i);
        const uint16_t base = m->base_addr;
        const uint8_t  size = m->size;

        if (m->per_motor) {
            for (uint8_t motor = 0; motor < (uint8_t)REG_MOTOR_COUNT; ++motor) {
                const uint16_t start = base + motor * size;
                for (uint8_t w = 0; w < size; ++w) {
                    const uint16_t addr = start + w;
                    if (addr < REG_ADDR_MIN || addr > REG_ADDR_MAX) continue;
                    const uint16_t idx = addr - REG_ADDR_MIN;
                    g_reg_lut[idx].reg_id = i;
                    g_reg_lut[idx].motor = motor;
                }
            }
        } else {
            for (uint8_t w = 0; w < size; ++w) {
                const uint16_t addr = (uint16_t)(base + w);
                if (addr < REG_ADDR_MIN || addr > REG_ADDR_MAX) continue;
                const uint16_t idx = (uint16_t)(addr - REG_ADDR_MIN);
                g_reg_lut[idx].reg_id = i;
                g_reg_lut[idx].motor = 0;
            }
        }
    }

    g_reg_lut_inited = true;
}

uint16_t reg_address_from_id(reg_id_t id, uint8_t motor) {
    const reg_meta_t* m = get_reg_meta(id);
    if (!m) return 0;

    if (m->per_motor) {
        if (motor >= (uint8_t)REG_MOTOR_COUNT) return 0;
        return m->base_addr + (uint16_t)motor * (uint16_t)m->size;
    }
    return m->base_addr;
}

reg_lut_t reg_id_from_address(uint16_t address) {
    // require reg_map_init()
    assert_param(g_reg_lut_inited);
    reg_map_init();

    if (address < REG_ADDR_MIN || address > REG_ADDR_MAX) return {REG_ID__NONE, 0};

    const reg_lut_t v = g_reg_lut[(uint16_t)(address - REG_ADDR_MIN)];
    return v;

}


bool reg_address_check_range(uint16_t address, uint16_t count){
    if(address < REG_ADDR_MIN || (address+count) > REG_ADDR_MAX) return false;
    return true;
}