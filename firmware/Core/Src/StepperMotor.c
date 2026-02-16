#include "StepperMotor.h"
#include <assert.h>
#include <memory.h>
#include "config.h"


//declarations functions
static void StepperMotor_update(StepperMotor* self);
static void StepperMotor_init(StepperMotor* self, TIM_TypeDef *timer, GPIO_TypeDef *dir_port, uint8_t dir_pin);




// link methods to class
static void StepperMotor_setAllmethods(StepperMotor* self){
	assert(self);
	self->update = StepperMotor_update;
	self->init = StepperMotor_init;

}


static inline int32_t _abs(int32_t x){ return x < 0 ? -x : x; }
static inline int32_t _min(int32_t a, int32_t b){ return a < b ? a : b; }
static inline int32_t _max(int32_t a, int32_t b){ return a > b ? a : b; }

// methods
static void StepperMotor_update(StepperMotor* m){
	StepperMotorParameters* p = &m->parameters;
	portENTER_CRITICAL();

	int8_t direction = (p->remaining_steps > 0) ? (1) : (-1);


	// Расчет дистанции торможения (правильная формула)
	// s = v² / (2 * a)
	int64_t brake_distance = 0;
	if (p->max_acceleration > 0) {
		brake_distance = ((int64_t)p->current_velocity * (int64_t)p->current_velocity);
		brake_distance /= (2 * p->max_acceleration);
	} else 
		brake_distance = INT64_MAX;
	int32_t abs_remaining = _abs(p->remaining_steps);

	

	// ускорение за тик + аккумулируем значение если дробное
	int32_t total_acc = p->max_acceleration + m->accelerationAccumulated; 
	int32_t accel_per_tick =  total_acc / 1000;
	m->accelerationAccumulated = total_acc % 1000;


	// Управление скоростью по трапециевидному профилю
	if ( abs_remaining <= brake_distance )
	{ // шагов нужно выполнить меньше чем нужно для тормаения
		//  ==> тормозим
		p->current_velocity = _max(0, p->current_velocity - accel_per_tick);		
	}
	else if (p->current_velocity < p->max_velocity){
		// или если это не максимальная скорость  ==> ускоряемся
		p->current_velocity = _min(p->max_velocity, p->current_velocity + accel_per_tick);		
	}
	else if (p->current_velocity > p->max_velocity)
	{ // если vmax изменили в меньшую сторону ==> тормозим
		p->current_velocity = _max(p->max_velocity, p->current_velocity - accel_per_tick);
	}


	// шаги/тик + аккумулируем значение если дробное
	int32_t total_vel = p->current_velocity + m->velocityAccumulated;
	int32_t vel_per_tick = total_vel / 1000;
	m->velocityAccumulated = total_vel % 1000;


	if (vel_per_tick != 0)
	{

		// НЕ допускать перепрыгивания через ноль:
		int32_t steps_to_move = vel_per_tick;
    if (steps_to_move > abs_remaining) steps_to_move = abs_remaining;

		if(steps_to_move > 256) steps_to_move = 256;

		//выбор направления двиения
		if(direction == 1){ 
			LL_GPIO_SetOutputPin(m->dir_port, m->dir_pin_Msk);
			p->remaining_steps -= steps_to_move;
			p->move_total_steps += steps_to_move;
		} else {
			LL_GPIO_ResetOutputPin(m->dir_port, m->dir_pin_Msk);
			p->remaining_steps += steps_to_move;
			p->move_total_steps -= steps_to_move;
		}
		if(p->remaining_steps == 0 ){
			//LL_GPIO_ResetOutputPin(m->dir_port, m->dir_pin_Msk);// дебаг
			p->current_velocity = 0;
			m->velocityAccumulated = 0;
			m->accelerationAccumulated = 0;
		}


		//LL_TIM_DisableCounter(m->timer);
		static const uint32_t timer_clk = 64000; // кГц
		LL_TIM_SetPrescaler(m->timer, ( timer_clk/(steps_to_move*2) ) );//- 1
    LL_TIM_SetRepetitionCounter(m->timer, steps_to_move - 1);
		
    LL_TIM_GenerateEvent_UPDATE(m->timer);
    LL_TIM_EnableCounter(m->timer);
		
	}

	portEXIT_CRITICAL();
}


static void StepperMotor_init(StepperMotor* self, TIM_TypeDef *timer, GPIO_TypeDef *dir_port, uint8_t dir_pin){
	assert(self);
	assert(timer);
	assert(dir_port);
	assert(dir_pin < 16);

	self->timer = timer;
	self->dir_port = dir_port;
	self->dir_pin_Msk = (1UL << dir_pin);

	LL_GPIO_ResetOutputPin(self->dir_port, self->dir_pin_Msk);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = self->dir_pin_Msk;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(self->dir_port, &GPIO_InitStruct);


/*
		assert(motor->timer);
		assert(IS_TIM_REPETITION_COUNTER_INSTANCE(motor->timer));

		LL_TIM_InitTypeDef TIM_InitStruct = {0};
		TIM_InitStruct.Prescaler = 63999;
		TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
		TIM_InitStruct.Autoreload = 1;
		TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
		TIM_InitStruct.RepetitionCounter = 0;
		LL_TIM_Init(motor->timer, &TIM_InitStruct);
*/



}










// functions for creation & destraction
StepperMotor* StepperMotor_create(){
	StepperMotor* motor = pvPortMalloc(sizeof(StepperMotor));
	assert(motor);
	memset(motor, 0, sizeof(StepperMotor));
	StepperMotor_setAllmethods(motor);
	motor->parameters.max_acceleration =128000;
	motor->parameters.max_velocity = 16000;
	return motor;
}

void StepperMotor_destroy(StepperMotor* motor){
	assert(motor);
	pvPortFree(motor);
}


