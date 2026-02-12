#pragma once
#include "FreeRTOS.h"
#include "task.h"

#include "StepperMotor.h"

#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"

#define MAX_STEPPER_MOTOR_COUNT 6




typedef struct StepperMotorController_ StepperMotorController;

struct StepperMotorController_ {
	void (*init)(StepperMotorController* self, TIM_TypeDef* timer);
	void (*startTimer)(StepperMotorController* self);
	void (*stopTimer)(StepperMotorController* self);
	void (*addMotor)(StepperMotorController* self, StepperMotor* motor);
	void (*updateMotors)(StepperMotorController* self);
	void (*updateMotorsISR)(StepperMotorController* self);
	void (*notifyTaskISR)(StepperMotorController* self, BaseType_t *pxHigherPriorityTaskWoken );

	

	StepperMotor* motors[MAX_STEPPER_MOTOR_COUNT];
	uint8_t motorCount;
	TIM_TypeDef *updateTimer;

	TaskHandle_t taskHandle;
};




StepperMotorController* StepperMotorController_create();
StepperMotorController* getStepperMotorController();






