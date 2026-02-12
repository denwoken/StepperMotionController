#include "FreeRTOS.h"

// 1. Критичные прерывания (самый высокий)
#define PRIORITY_CRITICAL_ISR    	(configMAX_PRIORITIES - 1)
#define PRIORITY_HIGH   					(configMAX_PRIORITIES - 3)
#define PRIORITY_ABOVE_NORMAL     (configMAX_PRIORITIES - 4)
#define PRIORITY_NORMAL          	(configMAX_PRIORITIES - 5)
#define PRIORITY_BELOW_NORMAL     (configMAX_PRIORITIES - 6)
#define PRIORITY_LOW     					(configMAX_PRIORITIES - 7)
#define PRIORITY_IDLE            	(tskIDLE_PRIORITY)





#define MODBUS_TASK_PRIORITY  PRIORITY_NORMAL
#define MOTOR_CONTROLLER_TIMER_ISR_PRIORITY PRIORITY_CRITICAL_ISR
