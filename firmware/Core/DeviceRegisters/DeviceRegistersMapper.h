#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "DeviceRegisters.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REGMAP_MAX_MOTORS MAX_STEPPER_MOTOR_COUNT

typedef struct {
    void *ptr;           /* points to uint16_t/int16_t/uint32_t/int32_t */
    reg_type_t type;     /* data type */
    reg_access_t access; /* optional override */
} reg_binding_t;

/* Word index for 32-bit values: 0 = low word, 1 = high word. */
typedef enum {
    REG_WORD_LO = 0,
    REG_WORD_HI = 1
} reg_word_index_t;

typedef struct {
    uint16_t words[2];
    uint8_t valid_mask; /* bit0: low, bit1: high */
} reg_u32_accum_t;

typedef struct {
    uint32_t value;
    uint8_t valid;
} reg_u32_latch_t;

typedef struct {
    const reg_binding_t *bindings; /* array size REG__COUNT */
    reg_u32_accum_t u32_accum[REG__COUNT];
    reg_u32_latch_t u32_latch[REG__COUNT];
} regmap_motor_t;

typedef struct {
    regmap_motor_t motors[REGMAP_MAX_MOTORS];
    uint8_t motor_count;
} regmap_t;



#ifdef __cplusplus
}
#endif
