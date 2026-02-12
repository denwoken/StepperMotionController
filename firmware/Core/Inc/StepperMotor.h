#pragma once
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g071xx.h"

typedef struct StepperMotor_ StepperMotor;


struct StepperMotor_ {
	//void (*move)(StepperMotor* self, int32_t steps);
	//void (*setVelocity)(StepperMotorPrivate* self, int32_t velocity);
	//void (*stop)(StepperMotor* self);
	void (*init)(StepperMotor* self, TIM_TypeDef *timer, GPIO_TypeDef *dir_port, uint8_t dir_pin);
	void (*update)(StepperMotor* self);
	// union
	// {
	// 	struct {


			uint32_t max_velocity;    	// максимальная (шаги/сек)
			uint32_t max_acceleration; // Ускорение (шаги/сек²)

			int32_t current_velocity;   // Реальная текущая скорость
			int32_t remaining_steps;    // Сколько шагов осталось в текущем перемещении
			int32_t move_total_steps; // Общее количество выполняных шагов

			int32_t velAccumulated;  // для точного подсчета шагов при малых скоростях
	// 	};
	// 	int32_t raw[5];
	// };

	int32_t accelerationAccumulated;  // остаток скорости с предыдущего тика -> учесть на следующем шаге
	int32_t velocityAccumulated;  // остаток скорости с предыдущего тика -> учесть на следующем шаге

	TIM_TypeDef *timer;
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin_Msk;
};


StepperMotor* StepperMotor_create();
void StepperMotor_destroy(StepperMotor* motor);







