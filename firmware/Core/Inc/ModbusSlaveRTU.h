#pragma once 
#include "slave.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdbool.h>


#include "stm32g0xx_ll_dma.h"
#include "usart.h"

/*
class ModbusSlaveRTU {
protected:
	ModbusSlaveRTU();
	~ModbusSlaveRTU();
public:
	ModbusSlaveRTU* Instance(){
		static ModbusSlaveRTU instance;
		return &instance;
	}


private: 
};
*/
/**/


typedef struct {
	ModbusSlave modbus;

	USART_TypeDef* 	uart;      // UART handle (для RX/TX)
	DMA_TypeDef*  	dma_rx;    // DMA handle RX (опционально)
	DMA_TypeDef*  	dma_tx;    // DMA handle TX (опционально)
	uint32_t dma_rx_channel;
	uint32_t dma_tx_channel;


	uint8_t rxBuf[MODBUS_RTU_ADU_MAX];
	uint8_t txBuf[MODBUS_RTU_ADU_MAX];
//	SemaphoreHandle_t mutex;			// мьютекс для защиты контекста
	SemaphoreHandle_t txDoneSem;  // сигнал от ISR о завершении передачи 
	SemaphoreHandle_t rxDoneSem;  // для сигнала о полном приёме через IDLE  

	uint8_t slaveAddress;
	volatile uint16_t rx_len;
	volatile uint16_t rx_buffer_size;

	TaskHandle_t taskHandle;
}ModbusRTU_t;

ModbusRTU_t* ModbusRTUNew();
ModbusRTU_t* ModbusRTUInstance();




void ModbusRTU_Init(ModbusRTU_t *ctx,
					USART_TypeDef *huart,
					DMA_TypeDef *dma_rx, uint32_t Channel_rx,
					DMA_TypeDef *dma_tx, uint32_t Channel_tx,
					uint8_t slaveAddress);


void ModbusRTU_Transmit(ModbusRTU_t *ctx, const uint8_t *data, uint16_t size);
void ModbusRTU_Receive(ModbusRTU_t *ctx, uint8_t *data, uint16_t maxSize, uint16_t* rxLen);


void ModbusRTU_dmaTxCpltCallback(ModbusRTU_t *ctx, BaseType_t *pxHigherPriorityTaskWoken);
void ModbusRTU_IDLECallback(ModbusRTU_t *ctx, BaseType_t *pxHigherPriorityTaskWoken);
