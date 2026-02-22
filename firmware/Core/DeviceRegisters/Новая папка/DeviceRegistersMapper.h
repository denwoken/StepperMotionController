#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "DeviceRegisters.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REGMAP_MAX_MOTORS MAX_STEPPER_MOTOR_COUNT

/* Word index for 32-bit values: 0 = low word, 1 = high word. */
typedef enum {
    REG_WORD_LO = 0,
    REG_WORD_HI = 1
} reg_word_index_t;

typedef struct {
    struct StepperMotorController_ *ctrl;
    uint8_t motor_count;
} regmap_t;

struct StepperMotorController_;

void regmap_bind_controller(regmap_t* map, struct StepperMotorController_ *ctrl);
void regmap_capture(const regmap_t* map);

reg_word_index_t regmap_word_index(uint16_t address, uint8_t motor, reg_id_t id);
bool regmap_req_covers_u32(uint16_t req_index, uint16_t req_count, uint8_t motor, reg_id_t id);

bool regmap_get_access(uint8_t motor, reg_id_t id, reg_access_t* out_access);
uint16_t regmap_read_snapshot(const regmap_t* map, uint8_t motor, reg_id_t id, reg_word_index_t word);
bool regmap_write_u16_direct(const regmap_t* map, uint8_t motor, reg_id_t id, uint16_t value);
bool regmap_write_u32_direct(const regmap_t* map, uint8_t motor, reg_id_t id, uint32_t value);

#ifdef __cplusplus
}
#endif
