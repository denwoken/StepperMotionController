#include "StepperMotorController.h"
#include "FreeRTOS.h"
#include "task.h"
#include <assert.h>
#include <memory.h>
#include "config.h"

//declarations functions
static void StepperMotorController_addMotor(StepperMotorController* self, StepperMotor* motor);
static void StepperMotorController_startTimer(StepperMotorController* self);
static void StepperMotorController_stopTimer(StepperMotorController* self);
static void StepperMotorController_init(StepperMotorController* self, TIM_TypeDef* timer);
static void StepperMotorController_updateMotors(StepperMotorController* self);
static void StepperMotorController_updateMotorsISR(StepperMotorController* self);
static void StepperMotorController_notifyTaskISR(StepperMotorController* self, BaseType_t *pxHigherPriorityTaskWoken );

static void StepperMotorController_Task(void* pvParameters);


// link methods to class
static void StepperMotorController_setAllmethods(StepperMotorController* self){
	assert(self);
	self->init = StepperMotorController_init;
	self->addMotor = StepperMotorController_addMotor;
	self->startTimer = StepperMotorController_startTimer;
	self->stopTimer = StepperMotorController_stopTimer;
	self->updateMotors = StepperMotorController_updateMotors;
	self->updateMotorsISR = StepperMotorController_updateMotorsISR;
	self->notifyTaskISR = StepperMotorController_notifyTaskISR;
}




// methods
static void StepperMotorController_addMotor(StepperMotorController* self, StepperMotor* motor){
	assert(self);
	assert(motor);

	if(self->motorCount >= MAX_STEPPER_MOTOR_COUNT){
		abort();
		return;
	}
	self->motors[self->motorCount] = motor;
	self->motorCount++;
}
static void StepperMotorController_init(StepperMotorController* self, TIM_TypeDef* timer){
	assert(self);
	assert(timer);

	self->updateTimer = timer;
	LL_TIM_InitTypeDef TIM_InitStruct = {0};
	TIM_InitStruct.Prescaler = 63;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1000;
  LL_TIM_Init(self->updateTimer, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(self->updateTimer);
  LL_TIM_SetTriggerOutput(self->updateTimer, LL_TIM_TRGO_RESET);//LL_TIM_TRGO_UPDATE
  LL_TIM_DisableMasterSlaveMode(self->updateTimer);
	LL_TIM_GenerateEvent_UPDATE(self->updateTimer);


	if(self->taskHandle) {
		vTaskDelete(self->taskHandle);
		self->taskHandle = NULL;
	}
	BaseType_t ret = 
	xTaskCreate (	
		(TaskFunction_t)StepperMotorController_Task,
		"StepperMotorController", 
		128, 
		self, 
		MOTOR_CONTROLLER_TIMER_ISR_PRIORITY, 
		&self->taskHandle
	);
	if(ret != pdPASS){
		printf("StepperMotorController task creation error %ld\r\n", ret);
		abort();
	}

}
static void StepperMotorController_startTimer(StepperMotorController* self){
	assert(self);
	assert(self->updateTimer);
	LL_TIM_EnableIT_UPDATE(self->updateTimer);
	LL_TIM_EnableCounter(self->updateTimer);
}
static void StepperMotorController_stopTimer(StepperMotorController* self){
	assert(self);
	assert(self->updateTimer);
	LL_TIM_DisableIT_UPDATE(self->updateTimer);
	LL_TIM_DisableCounter(self->updateTimer);
}
static void StepperMotorController_updateMotors(StepperMotorController* self){
	assert(self);
	for(int i = 0; i < self->motorCount; i++){
		StepperMotor* motor = self->motors[i];
		assert(motor);
		motor->update(motor);
	}
}
static void StepperMotorController_updateMotorsISR(StepperMotorController* self){
	assert(self);
	if(LL_TIM_IsActiveFlag_UPDATE(self->updateTimer)){
    LL_TIM_ClearFlag_UPDATE(self->updateTimer);
		StepperMotorController_updateMotors(self);
  }
}
static void StepperMotorController_notifyTaskISR(StepperMotorController* self, BaseType_t *pxHigherPriorityTaskWoken ){
	assert(self);
	if(LL_TIM_IsActiveFlag_UPDATE(self->updateTimer))
	{
    LL_TIM_ClearFlag_UPDATE(self->updateTimer);
		vTaskNotifyGiveFromISR(self->taskHandle, pxHigherPriorityTaskWoken);
	}
}


static void StepperMotorController_Task(void* pvParameters){
	StepperMotorController* controller = (StepperMotorController*)pvParameters;
	assert(controller);
	//uint8_t buff[100];
	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		// for(int i = 0; i < sizeof(buff); i++) {  // Правильное условие
    //   buff[i] = i;  // Простая операция
    // }
		controller->updateMotors(controller);
	}
}





// functions for creation & destraction
StepperMotorController* StepperMotorController_create(){
	StepperMotorController* controller = pvPortMalloc(sizeof(StepperMotorController));
	assert(controller);
	memset(controller, 0, sizeof(StepperMotorController));
	StepperMotorController_setAllmethods(controller);
	return controller;
}

StepperMotorController* getStepperMotorController()
{
	static StepperMotorController* s_controller = NULL;
	if(!s_controller){
		s_controller = StepperMotorController_create();
	}
	return s_controller;
}




