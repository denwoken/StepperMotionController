/* Auto-generated from Excel. Do not edit manually. Edit in *.xlsx */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define REG_BASE_ADDRESS 40000U
#define REG_MOTOR_STRIDE_WORDS 64U
#define REG_MOTOR_BLOCK_START 16U

/* Register access level: read/write/read-write */
typedef enum {
    REG_ACC_R  = 1,
    REG_ACC_W  = 2,
    REG_ACC_RW = 3
} reg_access_t;

/* Register data type */
typedef enum {
    REG_T_U16 = 1,
    REG_T_I16 = 2,
    REG_T_U32 = 3,
    REG_T_I32 = 4
} reg_type_t;

typedef enum {
    REG_CONTROL = 0,
    REG_STATUS = 1,
    REG_ERROR_CODE = 2,
    REG_MODE = 3,
    REG_CMD = 4,
    REG_CURRENT_POS_16 = 5,
    REG_CURRENT_VELOCITY_16 = 6,
    REG_CURRENT_ACCEL_16 = 7,
    REG_TARGET_POS_16 = 8,
    REG_MAX_VELOCITY_16 = 9,
    REG_MAX_ACCEL_16 = 10,
    REG_MOVE_POS_REL_16 = 11,
    REG_CURRENT_POS_32 = 12,
    REG_CURRENT_VELOCITY_32 = 13,
    REG_CURRENT_ACCEL_32 = 14,
    REG_TARGET_POS_32 = 15,
    REG_MAX_VELOCITY_32 = 16,
    REG_MAX_ACCEL_32 = 17,
    REG_MOVE_POS_REL_32 = 18,
    REG__COUNT = 19
} reg_id_t;

/* Optional: offsets inside motor block (in 16-bit registers) */
#define REG_MOTOR_OFFSET_CONTROL              (0u)
#define REG_MOTOR_OFFSET_STATUS               (1u)
#define REG_MOTOR_OFFSET_ERROR_CODE           (2u)
#define REG_MOTOR_OFFSET_MODE                 (3u)
#define REG_MOTOR_OFFSET_CMD                  (4u)
#define REG_MOTOR_OFFSET_CURRENT_POS_16       (16u)
#define REG_MOTOR_OFFSET_CURRENT_VELOCITY_16  (17u)
#define REG_MOTOR_OFFSET_CURRENT_ACCEL_16     (18u)
#define REG_MOTOR_OFFSET_TARGET_POS_16        (19u)
#define REG_MOTOR_OFFSET_MAX_VELOCITY_16      (20u)
#define REG_MOTOR_OFFSET_MAX_ACCEL_16         (21u)
#define REG_MOTOR_OFFSET_MOVE_POS_REL_16      (22u)
#define REG_MOTOR_OFFSET_CURRENT_POS_32       (32u)
#define REG_MOTOR_OFFSET_CURRENT_VELOCITY_32  (34u)
#define REG_MOTOR_OFFSET_CURRENT_ACCEL_32     (36u)
#define REG_MOTOR_OFFSET_TARGET_POS_32        (38u)
#define REG_MOTOR_OFFSET_MAX_VELOCITY_32      (40u)
#define REG_MOTOR_OFFSET_MAX_ACCEL_32         (42u)
#define REG_MOTOR_OFFSET_MOVE_POS_REL_32      (44u)

#define REG_INDEX_CONTROL              (16u)
#define REG_INDEX_STATUS               (17u)
#define REG_INDEX_ERROR_CODE           (18u)
#define REG_INDEX_MODE                 (19u)
#define REG_INDEX_CMD                  (20u)
#define REG_INDEX_CURRENT_POS_16       (32u)
#define REG_INDEX_CURRENT_VELOCITY_16  (33u)
#define REG_INDEX_CURRENT_ACCEL_16     (34u)
#define REG_INDEX_TARGET_POS_16        (35u)
#define REG_INDEX_MAX_VELOCITY_16      (36u)
#define REG_INDEX_MAX_ACCEL_16         (37u)
#define REG_INDEX_MOVE_POS_REL_16      (38u)
#define REG_INDEX_CURRENT_POS_32       (48u)
#define REG_INDEX_CURRENT_VELOCITY_32  (50u)
#define REG_INDEX_CURRENT_ACCEL_32     (52u)
#define REG_INDEX_TARGET_POS_32        (54u)
#define REG_INDEX_MAX_VELOCITY_32      (56u)
#define REG_INDEX_MAX_ACCEL_32         (58u)
#define REG_INDEX_MOVE_POS_REL_32      (60u)

typedef struct {
    reg_id_t     id;
    uint16_t     index;      /* index in motor block (16-bit registers) */
    uint16_t     size;       /* size in bytes */
    reg_type_t   type;       /* reg data type */
    reg_access_t access;
    const char*  name;
    const char*  units;
} reg_meta_t;

/* Metadata for every register of the device. */
extern const reg_meta_t g_reg_meta[REG__COUNT];

/* Get absolute Modbus address for motor (motor is 0..N). */
static inline uint32_t reg_address(uint8_t motor, reg_id_t id)
{
    /* address = BASE + motor offset + motor*STRIDE + index */
    return (uint32_t)REG_BASE_ADDRESS + (uint32_t)REG_MOTOR_BLOCK_START + 
           (uint32_t)(motor * (uint32_t)REG_MOTOR_STRIDE_WORDS) + (uint32_t)g_reg_meta[id].index;
}

const reg_meta_t* reg_get_meta(reg_id_t id);

