/* Auto-generated from Excel. Do not edit manually. */
#include "DeviceRegisters.h"
#include <assert.h>

const reg_meta_t g_reg_meta[REG__COUNT] = {
    {
        .id = REG_CONTROL,
        .motorReg = true,
        .index = 0u,
        .size = 2u,
        .type = REG_T_U16,
        .access = REG_ACC_RW,
        .name = "CONTROL",
        .units = "bitfield",
    },
    {
        .id = REG_STATUS,
        .motorReg = true,
        .index = 1u,
        .size = 2u,
        .type = REG_T_U16,
        .access = REG_ACC_R,
        .name = "STATUS",
        .units = "bitfield",
    },
    {
        .id = REG_ERROR_CODE,
        .motorReg = true,
        .index = 2u,
        .size = 2u,
        .type = REG_T_U16,
        .access = REG_ACC_R,
        .name = "ERROR_CODE",
        .units = "",
    },
    {
        .id = REG_MODE,
        .motorReg = true,
        .index = 3u,
        .size = 2u,
        .type = REG_T_U16,
        .access = REG_ACC_RW,
        .name = "MODE",
        .units = "",
    },
    {
        .id = REG_CMD,
        .motorReg = true,
        .index = 4u,
        .size = 2u,
        .type = REG_T_U16,
        .access = REG_ACC_W,
        .name = "CMD",
        .units = "",
    },
    {
        .id = REG_CURRENT_POS_16,
        .motorReg = true,
        .index = 16u,
        .size = 2u,
        .type = REG_T_I16,
        .access = REG_ACC_R,
        .name = "CURRENT_POS_16",
        .units = "step",
    },
    {
        .id = REG_CURRENT_VELOCITY_16,
        .motorReg = true,
        .index = 17u,
        .size = 2u,
        .type = REG_T_I16,
        .access = REG_ACC_R,
        .name = "CURRENT_VELOCITY_16",
        .units = "8 step/s",
    },
    {
        .id = REG_CURRENT_ACCEL_16,
        .motorReg = true,
        .index = 18u,
        .size = 2u,
        .type = REG_T_I16,
        .access = REG_ACC_R,
        .name = "CURRENT_ACCEL_16",
        .units = "256 step/sВІ",
    },
    {
        .id = REG_TARGET_POS_16,
        .motorReg = true,
        .index = 19u,
        .size = 2u,
        .type = REG_T_I16,
        .access = REG_ACC_RW,
        .name = "TARGET_POS_16",
        .units = "step",
    },
    {
        .id = REG_MAX_VELOCITY_16,
        .motorReg = true,
        .index = 20u,
        .size = 2u,
        .type = REG_T_U16,
        .access = REG_ACC_RW,
        .name = "MAX_VELOCITY_16",
        .units = "8 step/s",
    },
    {
        .id = REG_MAX_ACCEL_16,
        .motorReg = true,
        .index = 21u,
        .size = 2u,
        .type = REG_T_U16,
        .access = REG_ACC_RW,
        .name = "MAX_ACCEL_16",
        .units = "256 step/sВІ",
    },
    {
        .id = REG_MOVE_POS_REL_16,
        .motorReg = true,
        .index = 22u,
        .size = 2u,
        .type = REG_T_I16,
        .access = REG_ACC_W,
        .name = "MOVE_POS_REL_16",
        .units = "step",
    },
    {
        .id = REG_CURRENT_POS_32,
        .motorReg = true,
        .index = 32u,
        .size = 4u,
        .type = REG_T_I32,
        .access = REG_ACC_R,
        .name = "CURRENT_POS_32",
        .units = "step",
    },
    {
        .id = REG_CURRENT_VELOCITY_32,
        .motorReg = true,
        .index = 34u,
        .size = 4u,
        .type = REG_T_I32,
        .access = REG_ACC_R,
        .name = "CURRENT_VELOCITY_32",
        .units = "step/sec",
    },
    {
        .id = REG_CURRENT_ACCEL_32,
        .motorReg = true,
        .index = 36u,
        .size = 4u,
        .type = REG_T_I32,
        .access = REG_ACC_R,
        .name = "CURRENT_ACCEL_32",
        .units = "step/secВІ",
    },
    {
        .id = REG_TARGET_POS_32,
        .motorReg = true,
        .index = 38u,
        .size = 4u,
        .type = REG_T_I32,
        .access = REG_ACC_RW,
        .name = "TARGET_POS_32",
        .units = "step",
    },
    {
        .id = REG_MAX_VELOCITY_32,
        .motorReg = true,
        .index = 40u,
        .size = 4u,
        .type = REG_T_U32,
        .access = REG_ACC_RW,
        .name = "MAX_VELOCITY_32",
        .units = "step/sec",
    },
    {
        .id = REG_MAX_ACCEL_32,
        .motorReg = true,
        .index = 42u,
        .size = 4u,
        .type = REG_T_U32,
        .access = REG_ACC_RW,
        .name = "MAX_ACCEL_32",
        .units = "step/secВІ",
    },
    {
        .id = REG_MOVE_POS_REL_32,
        .motorReg = true,
        .index = 44u,
        .size = 4u,
        .type = REG_T_I32,
        .access = REG_ACC_W,
        .name = "MOVE_POS_REL_32",
        .units = "step",
    },
};

const reg_meta_t* get_reg_meta_by_id(const reg_id_t id)
{
    assert((int)id >= 0 && id < REG__COUNT);
    return &g_reg_meta[id];
}
static reg_id_t g_offset_to_general_reg_id[REG_MOTOR_BLOCK_START];
static reg_id_t g_offset_to_motor_reg_id[REG_MOTOR_STRIDE_WORDS];
static bool g_offset_to_reg_id_init = false;

static void reg_build_offset_maps(void)
{
    if (g_offset_to_reg_id_init) {
        return;
    }

    for (uint16_t i = 0u; i < (uint16_t)REG_MOTOR_BLOCK_START; i++) {
        g_offset_to_general_reg_id[i] = REG__RESERVED;
    }
    for (uint16_t i = 0u; i < (uint16_t)REG_MOTOR_STRIDE_WORDS; i++) {
        g_offset_to_motor_reg_id[i] = REG__RESERVED;
    }


    for (reg_id_t i = 0; i < REG__COUNT; i++) {
        const uint16_t idx = g_reg_meta[i].index;
        const uint16_t words = (uint16_t)(g_reg_meta[i].size / 2u);
        if (words == 0u) continue;

        const bool is_motor = g_reg_meta[i].motorReg;
        for (uint16_t w = 0u; w < words; w++) {
            const uint16_t off = (uint16_t)(idx + w);
            if (!is_motor) {
                /* General registers */
                if (off < (uint16_t)REG_MOTOR_BLOCK_START) {
                    g_offset_to_general_reg_id[off] = (reg_id_t)i;
                }
            } else {
                /* Motor registers */
                if (off < (uint16_t)REG_MOTOR_STRIDE_WORDS) {
                    g_offset_to_motor_reg_id[off] = (reg_id_t)i;
                }
            }
        }
    }

    g_offset_to_reg_id_init = true;
}

const reg_id_t get_reg_id_by_address(uint16_t address,
                                     uint8_t *out_motor)
{
    reg_build_offset_maps();

    uint32_t addr = (uint32_t)address;

    /* Accept absolute Modbus address (e.g. 40000+) or index. */
    if (addr >= (uint32_t)REG_BASE_ADDRESS) {
        addr -= (uint32_t)REG_BASE_ADDRESS;
    }

    if (addr < (uint32_t)REG_MOTOR_BLOCK_START) {
        /* GENERAL section */
#if ((REG_MOTOR_BLOCK_START & (REG_MOTOR_BLOCK_START - 1u)) == 0u)
        const uint16_t offset = (uint16_t)(addr & (uint32_t)(REG_MOTOR_BLOCK_START - 1u));
#else
        const uint16_t offset = (uint16_t)(addr % (uint32_t)REG_MOTOR_BLOCK_START);
#endif
        assert((offset >= 0) && 
            (offset < 
            (sizeof(g_offset_to_general_reg_id)/sizeof(reg_id_t)))
        );
        const reg_id_t id = g_offset_to_general_reg_id[offset];
        return id;
    }

    addr -= (uint32_t)REG_MOTOR_BLOCK_START;

#if ((REG_MOTOR_STRIDE_WORDS & (REG_MOTOR_STRIDE_WORDS - 1u)) == 0u)
    const uint16_t offset = (uint16_t)(addr & (uint32_t)(REG_MOTOR_STRIDE_WORDS - 1u));
#else
    const uint16_t offset = (uint16_t)(addr % (uint32_t)REG_MOTOR_STRIDE_WORDS);
#endif
    assert((offset >= 0) && 
        (offset < 
        (sizeof(g_offset_to_motor_reg_id)/sizeof(reg_id_t)))
    );
    const reg_id_t id = g_offset_to_motor_reg_id[offset];

    if(out_motor){
#if ((REG_MOTOR_STRIDE_WORDS & (REG_MOTOR_STRIDE_WORDS - 1u)) == 0u)
        const uint8_t motorNum = (uint16_t)(addr >> REG_MOTOR_STRIDE_WORDS_BITS);
#else
        const uint8_t motorNum = (uint16_t)(addr / (uint32_t)REG_MOTOR_STRIDE_WORDS);
#endif   
        *out_motor = motorNum;
    }
    return id;

}

const reg_id_t get_id_by_address(uint16_t address)
{
    reg_id_t id = REG__NONE;
    (void)reg_get_meta_by_address(address, NULL, &id);
    return id;
}


