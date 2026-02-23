#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Auto-generated from "DeviceRegisters.xlsx"

#define REG_BASE_ADDRESS	(40000U)
#define REG_COMMON_START	(0U)
#define REG_MOTOR_CONTROL_OFFSET	(8U)
#define REG_MOTOR32_OFFSET	(32U)
#define REG_MOTOR_COUNT	(5U)
#define REG_ADDR_MIN	(40000U)
#define REG_ADDR_MAX	(40101U)
#define REG_ADDR_SPAN	(102U)

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

/* Register IDs */
typedef enum {
    REG_ID__NONE = -2,
    REG_ID__RESERVED = -1,

    REG_ID_DEVICE_ID = 0,
    REG_ID_FW_VERSION = 1,
    REG_ID_MOTOR_COUNT = 2,
    REG_ID_CONTROL = 3,
    REG_ID_STATUS = 4,
    REG_ID_ERROR_CODE = 5,
    REG_ID_MODE = 6,
    REG_ID_CURRENT_POS_32 = 7,
    REG_ID_CURRENT_VELOCITY_32 = 8,
    REG_ID_CURRENT_ACCEL_32 = 9,
    REG_ID_TARGET_POS_32 = 10,
    REG_ID_MOVE_POS_REL_32 = 11,
    REG_ID_MAX_VELOCITY_32 = 12,
    REG_ID_MAX_ACCEL_32 = 13,

    REG_ID__COUNT = 14
} reg_id_t;


typedef struct {
    reg_id_t  id;
    uint16_t  base_addr;  // Modbus holding register address (word address)
    uint8_t   words;      // Size in 16-bit words (1 for 16-bit, 2 for 32-bit)
    uint8_t   per_motor;  // 1 if indexed by motor
    reg_access_t access;
    reg_type_t dtype;     // reg data type
} reg_meta_t;

typedef struct {
    reg_id_t reg_id;
    uint8_t motor;
} reg_lut_t;

void reg_map_init(void);
reg_lut_t reg_id_from_address(uint16_t address);
uint16_t reg_address_from_id(reg_id_t id, uint8_t motor);
const reg_meta_t* get_reg_meta(reg_id_t id);
bool reg_address_check_range(uint16_t address, uint16_t count);

#ifdef __cplusplus
}
#endif
