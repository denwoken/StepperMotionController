#include "ModbusSlaveRTU.h"

#include "modbus.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "stm32g0xx_ll_dma.h"
#include "usart.h"

#include <assert.h>
#include "config.h"

#define HREG_COUNT 16 //Numer of Hold Registers
#define COIL_COUNT 16 //Numer of Coils
#define DINP_COUNT 16 //Numer of Discrete Inputs
#define IREG_COUNT 16 //Numer of (Input Registers



  // Регистры ModBus
  uint16_t hregs[HREG_COUNT];
  uint8_t  coils[COIL_COUNT/8];
  uint16_t iregs[IREG_COUNT];
  uint8_t  dinpt[DINP_COUNT/8];



ModbusError registerCallback(
  const ModbusSlave *slave,
  const ModbusRegisterCallbackArgs *args,
  ModbusRegisterCallbackResult *result)
{
  // printf("Modbus registerCallback: type=%d, query=%d, index=%d, value=%d, function=%d\r\n",
  //   args->type, args->query, args->index, args->value, args->function);

  switch (args->query)
  {
	// All regs can be read
	case MODBUS_REGQ_R_CHECK:
	if (args->index < HREG_COUNT && args->type == MODBUS_HOLDING_REGISTER)	result->exceptionCode = MODBUS_EXCEP_NONE;
	else  if (args->index < IREG_COUNT && args->type == MODBUS_INPUT_REGISTER)	result->exceptionCode = MODBUS_EXCEP_NONE;
	else  if (args->index < COIL_COUNT && args->type == MODBUS_COIL)	result->exceptionCode = MODBUS_EXCEP_NONE;
	else  if (args->index < DINP_COUNT && args->type == MODBUS_DISCRETE_INPUT)	result->exceptionCode = MODBUS_EXCEP_NONE;
	else	result->exceptionCode = MODBUS_EXCEP_ILLEGAL_ADDRESS;
	break;
	  
	// All but two last regs/coils can be written
   case MODBUS_REGQ_W_CHECK:
	  // if (args->index < REG_COUNT - 2)
	if (args->index < HREG_COUNT && args->type == MODBUS_HOLDING_REGISTER)	result->exceptionCode = MODBUS_EXCEP_NONE;
	else  if (args->index < COIL_COUNT && args->type == MODBUS_COIL)	result->exceptionCode = MODBUS_EXCEP_NONE;
	else	result->exceptionCode = MODBUS_EXCEP_ILLEGAL_ADDRESS;
	break;

	// Read registers
	case MODBUS_REGQ_R:
	  switch (args->type)
	  {
		case MODBUS_HOLDING_REGISTER: result->value = hregs[args->index]; break;
		case MODBUS_INPUT_REGISTER: result->value = iregs[args->index]; break;
		case MODBUS_COIL: result->value = modbusMaskRead(coils, args->index); break;
		case MODBUS_DISCRETE_INPUT: result->value = modbusMaskRead(dinpt, args->index); break;
	  }
	  break;

	// Write registers
	case MODBUS_REGQ_W:
	  switch (args->type)
	  {
		case MODBUS_HOLDING_REGISTER:
		{ 
		  hregs[args->index] = args->value;
		  break;
		  }
		case MODBUS_COIL: modbusMaskWrite(coils, args->index, args->value); break;
		default: abort(); break;
	  }
	  break;
  }
	//GPIOA->ODR=coils[0];
  return MODBUS_OK;
}




static ModbusError slaveExceptionCallback(const ModbusSlave *slave, uint8_t function, ModbusExceptionCode code)
{
  (void)slave; (void)function; (void)code;
  printf("Modbus slave exception: function=%d, code=%d\r\n", function, code);
  return MODBUS_OK;// Always return MODBUS_OK
}

static ModbusError staticAllocator(
  ModbusBuffer *buffer,
  uint16_t size,
  void *context)
{
	ModbusRTU_t *ctx = (ModbusRTU_t*)context;
	if(ctx){
		if (size == 0) { // free request
        buffer->data = NULL;
        return MODBUS_OK;
    }
    if (size <= MODBUS_RTU_ADU_MAX) { // проверка на размер
        buffer->data = ctx->txBuf;    // используем буфер из структуры
        return MODBUS_OK;
    } else {
        buffer->data = NULL;
        return MODBUS_ERROR_ALLOC;
    }
	}
	return MODBUS_ERROR_ALLOC;
}






static modbusSlaveRTU_task(void* pvParameters){
	ModbusRTU_t *ctx = (ModbusRTU_t *)pvParameters;
	assert(ctx);

	// while(1){		// тест uart + dma
	// 	uint16_t recv_len = 0;
	// 	ModbusRTU_Receive(ctx, ctx->rxBuf, sizeof(ctx->rxBuf), &recv_len);
	// 	memcpy(ctx->txBuf, ctx->rxBuf, recv_len);
	// 	ModbusRTU_Transmit(ctx, ctx->txBuf, recv_len);
	// }
	
	while(1){

		ModbusErrorInfo err;

		uint16_t recv_len = 0;
		ModbusRTU_Receive(ctx, ctx->rxBuf, sizeof(ctx->rxBuf), &recv_len);
	
		//memset(buff, 0, sizeof(buff));
		//memcpy(buff, buf, len);
		//printf("modbus request: %s, len=%d\r\n", buff, len);
		
		//taskENTER_CRITICAL(); // защита регистров на время парсинга
		// ParceTime 50...70uS /33...44uS
		err = modbusParseRequestRTU(&ctx->modbus,1, (uint8_t *)ctx->rxBuf, recv_len);
		//taskEXIT_CRITICAL();

		if (modbusIsOk(err)) {
			// Отправка ответа
			uint8_t *resp = (uint8_t *)modbusSlaveGetResponse(&ctx->modbus);
			uint16_t resp_len = modbusSlaveGetResponseLength(&ctx->modbus);

			ModbusRTU_Transmit(ctx, resp, resp_len);
		}
		else printf("modbus parse error: %d\r\n", err);

	}
}

ModbusRTU_t* ModbusRTUNew(){
	ModbusRTU_t* modbus = pvPortMalloc(sizeof(ModbusRTU_t));
	assert(modbus);
	memset(modbus, 0, sizeof(ModbusRTU_t));
	return modbus;
}

ModbusRTU_t* ModbusRTUInstance(){
	static ModbusRTU_t* modbus;
	if(!modbus){
		modbus = ModbusRTUNew();
	}
	return modbus;
}


static inline uint32_t Modbus_GetDMAMUX_Request_RX(USART_TypeDef *uart)
{
    if(uart == USART1) return LL_DMAMUX_REQ_USART1_RX;
    if(uart == USART2) return LL_DMAMUX_REQ_USART2_RX;
    if(uart == USART3) return LL_DMAMUX_REQ_USART3_RX;
#ifdef USART4
    if(uart == USART4) return LL_DMAMUX_REQ_USART4_RX;
#endif
    // fallback (should not happen)
    return LL_DMAMUX_REQ_MEM2MEM;
}

static inline uint32_t Modbus_GetDMAMUX_Request_TX(USART_TypeDef *uart)
{
    if(uart == USART1) return LL_DMAMUX_REQ_USART1_TX;
    if(uart == USART2) return LL_DMAMUX_REQ_USART2_TX;
    if(uart == USART3) return LL_DMAMUX_REQ_USART3_TX;
#ifdef USART4
    if(uart == USART4) return LL_DMAMUX_REQ_USART4_TX;
#endif
    return LL_DMAMUX_REQ_MEM2MEM;
}


void ModbusRTU_Init(ModbusRTU_t *ctx,
					USART_TypeDef *uart,
					DMA_TypeDef *dma_rx, uint32_t Channel_rx,
					DMA_TypeDef *dma_tx, uint32_t Channel_tx,
					uint8_t slaveAddress)
{
	assert(ctx);
	memset(ctx, 0, sizeof(ModbusRTU_t));

	assert(uart);
	ctx->uart = uart;
	
	
	ctx->dma_rx = dma_rx;
	ctx->dma_tx = dma_tx;
	ctx->dma_rx_channel = Channel_rx;
	ctx->dma_tx_channel = Channel_tx; 
	
	ctx->slaveAddress = slaveAddress;

	ctx->txDoneSem = xSemaphoreCreateBinary();
	ctx->rxDoneSem = xSemaphoreCreateBinary();


	if(ctx->dma_rx){
		LL_DMA_DisableChannel(ctx->dma_rx, ctx->dma_rx_channel);

		LL_DMA_SetPeriphRequest(ctx->dma_rx, ctx->dma_rx_channel, Modbus_GetDMAMUX_Request_RX(ctx->uart));
		LL_DMA_SetDataTransferDirection(ctx->dma_rx, ctx->dma_rx_channel, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
		LL_DMA_SetMemoryIncMode(ctx->dma_rx, ctx->dma_rx_channel, LL_DMA_MEMORY_INCREMENT);
		LL_DMA_SetPeriphIncMode(ctx->dma_rx, ctx->dma_rx_channel, LL_DMA_PERIPH_NOINCREMENT);
		LL_DMA_SetChannelPriorityLevel(ctx->dma_rx, ctx->dma_rx_channel, LL_DMA_PRIORITY_LOW);
		LL_DMA_SetMode(ctx->dma_rx, ctx->dma_rx_channel, LL_DMA_MODE_NORMAL);
		LL_DMA_SetPeriphSize(ctx->dma_rx, ctx->dma_rx_channel, LL_DMA_PDATAALIGN_BYTE);
  	LL_DMA_SetMemorySize(ctx->dma_rx, ctx->dma_rx_channel, LL_DMA_MDATAALIGN_BYTE);
		uint32_t uart_dr = LL_USART_DMA_GetRegAddr(ctx->uart, LL_USART_DMA_REG_DATA_RECEIVE);
		LL_DMA_SetPeriphAddress(ctx->dma_rx, ctx->dma_rx_channel, uart_dr);

		// НЕ включаем канал здесь — будем запускать при старте приема
        // Включим IDLE IT у UART где нужно: LL_USART_EnableIT_IDLE(ctx->uart);
		//LL_DMA_EnableIT_TC(ctx->dma_tx, ctx->dma_tx_channel); // Включаем прерывание по завершению передачи
	}

	if(ctx->dma_tx){
		LL_DMA_DisableChannel(ctx->dma_tx, ctx->dma_tx_channel);
		
		LL_DMA_SetPeriphRequest(ctx->dma_tx, ctx->dma_tx_channel, Modbus_GetDMAMUX_Request_TX(ctx->uart));
		LL_DMA_SetDataTransferDirection(ctx->dma_tx, ctx->dma_tx_channel, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		LL_DMA_SetMemoryIncMode(ctx->dma_tx, ctx->dma_tx_channel, LL_DMA_MEMORY_INCREMENT);
		LL_DMA_SetPeriphIncMode(ctx->dma_tx, ctx->dma_tx_channel, LL_DMA_PERIPH_NOINCREMENT);
		LL_DMA_SetChannelPriorityLevel(ctx->dma_tx, ctx->dma_tx_channel, LL_DMA_PRIORITY_LOW);
		LL_DMA_SetMode(ctx->dma_tx, ctx->dma_tx_channel, LL_DMA_MODE_NORMAL);
		LL_DMA_SetPeriphSize(ctx->dma_tx, ctx->dma_tx_channel, LL_DMA_PDATAALIGN_BYTE);
  	LL_DMA_SetMemorySize(ctx->dma_tx, ctx->dma_tx_channel, LL_DMA_MDATAALIGN_BYTE);
		uint32_t uart_dr = LL_USART_DMA_GetRegAddr(ctx->uart, LL_USART_DMA_REG_DATA_TRANSMIT);
		LL_DMA_SetPeriphAddress(ctx->dma_tx, ctx->dma_tx_channel, uart_dr);
		
		// НЕ включаем канал здесь — будем запускать при старте передачи
		// включаем прерывание по TC для TX
    LL_DMA_EnableIT_TC(ctx->dma_tx, ctx->dma_tx_channel);
	}



	ModbusErrorInfo 
	err = modbusSlaveInit(
		&ctx->modbus,
		registerCallback,
		slaveExceptionCallback,
		staticAllocator,
		modbusSlaveDefaultFunctions,
		modbusSlaveDefaultFunctionCount);
	modbusSlaveSetUserPointer(&ctx->modbus, ctx);
	
	assert(modbusIsOk(err));
	

	if(ctx->taskHandle) {
		vTaskDelete(ctx->taskHandle);
		ctx->taskHandle = NULL;
	}
	BaseType_t ret = 
	xTaskCreate(
		modbusSlaveRTU_task, 
		"ModbusRTU", 
		128, 
		ctx, 
		MODBUS_TASK_PRIORITY, 
		&ctx->taskHandle
	);
	if(ret != pdPASS){
		printf("ModbusRTU task creation error %ld\r\n", ret);
		abort();
	}
}







static inline void DMA_ClearFlag_TC(DMA_TypeDef *DMAx, uint32_t channel)
{
    switch(channel)
    {
    case LL_DMA_CHANNEL_1: LL_DMA_ClearFlag_TC1(DMAx); break;
    case LL_DMA_CHANNEL_2: LL_DMA_ClearFlag_TC2(DMAx); break;
    case LL_DMA_CHANNEL_3: LL_DMA_ClearFlag_TC3(DMAx); break;
    case LL_DMA_CHANNEL_4: LL_DMA_ClearFlag_TC4(DMAx); break;
    case LL_DMA_CHANNEL_5: LL_DMA_ClearFlag_TC5(DMAx); break;
    case LL_DMA_CHANNEL_6: LL_DMA_ClearFlag_TC6(DMAx); break;
    case LL_DMA_CHANNEL_7: LL_DMA_ClearFlag_TC7(DMAx); break;
    default: break;
    }
}

static inline uint32_t DMA_IsActiveFlag_TC(DMA_TypeDef *DMAx, uint32_t channel)
{
    switch(channel)
    {
    case LL_DMA_CHANNEL_1: return LL_DMA_IsActiveFlag_TC1(DMAx);
    case LL_DMA_CHANNEL_2: return LL_DMA_IsActiveFlag_TC2(DMAx);
    case LL_DMA_CHANNEL_3: return LL_DMA_IsActiveFlag_TC3(DMAx);
    case LL_DMA_CHANNEL_4: return LL_DMA_IsActiveFlag_TC4(DMAx);
    case LL_DMA_CHANNEL_5: return LL_DMA_IsActiveFlag_TC5(DMAx);
    case LL_DMA_CHANNEL_6: return LL_DMA_IsActiveFlag_TC6(DMAx);
    case LL_DMA_CHANNEL_7: return LL_DMA_IsActiveFlag_TC7(DMAx);
    default: return 0;
    }
}







static uint16_t ModbusRTU_dmaRxCplt_wait(ModbusRTU_t *ctx){ 
	assert(ctx);
	xSemaphoreTake(ctx->rxDoneSem, portMAX_DELAY);
	return ctx->rx_len;
}

static void ModbusRTU_dmaTxCplt_wait(ModbusRTU_t *ctx){
	assert(ctx);
	xSemaphoreTake(ctx->txDoneSem, portMAX_DELAY);
}

void ModbusRTU_Transmit(ModbusRTU_t *ctx, const uint8_t *data, uint16_t size)
{
	assert(ctx);
	assert(data);

	if (ctx->dma_tx == NULL) 
    {
 		for(uint16_t i = 0; i < size; i++)
        {
            while(!LL_USART_IsActiveFlag_TXE_TXFNF(ctx->uart));
            LL_USART_TransmitData8(ctx->uart, data[i]);
        }
		while(!LL_USART_IsActiveFlag_TC(ctx->uart));

	}
	else{
		LL_DMA_DisableChannel(ctx->dma_tx, ctx->dma_tx_channel);   // отключаем перед конфигом
		/* Сброс флага завершения */
		DMA_ClearFlag_TC(ctx->dma_tx, ctx->dma_tx_channel);

		LL_DMA_SetDataLength(ctx->dma_tx, ctx->dma_tx_channel, size);
		LL_DMA_SetMemoryAddress(ctx->dma_tx, ctx->dma_tx_channel, (uint32_t)data);

		
    	LL_DMA_EnableChannel(ctx->dma_tx, ctx->dma_tx_channel); // Включаем DMA
		LL_USART_EnableDMAReq_TX(ctx->uart);   // Соединяем UART с DMA

		ModbusRTU_dmaTxCplt_wait(ctx); // Ждем окончания передачи
	}
}
void ModbusRTU_Receive(ModbusRTU_t *ctx, uint8_t *data, uint16_t maxSize, uint16_t* rxLen){
	assert(ctx);
	assert(data);
	
	ctx->rx_buffer_size = maxSize; // для прерывания IDLE
	LL_USART_ClearFlag_IDLE(ctx->uart);

	// clear RX FIFO
	while(LL_USART_IsActiveFlag_RXNE_RXFNE(ctx->uart)) {
    volatile uint8_t dummy = LL_USART_ReceiveData8(ctx->uart);
    (void)dummy;
	}

	if(ctx->dma_rx == NULL){
		for(uint16_t i = 0; i < maxSize; i++)
		{
			while(!LL_USART_IsActiveFlag_RXNE_RXFNE(ctx->uart)){
				// Если пришёл IDLE — значит кадр закончен
        if(LL_USART_IsActiveFlag_IDLE(ctx->uart))
        {
            LL_USART_ClearFlag_IDLE(ctx->uart);
            ctx->rx_len = i;
            *rxLen = ctx->rx_len;
            return;
        }
			}
			data[i] = LL_USART_ReceiveData8(ctx->uart);
			if(i == maxSize - 1){
				ctx->rx_len = maxSize;
				*rxLen = ctx->rx_len;
				return;
			}
		}
	}
	else
	{
		LL_DMA_DisableChannel(ctx->dma_rx, ctx->dma_rx_channel);   // отключаем перед конфигом

		LL_USART_ClearFlag_FE(ctx->uart);
		LL_USART_ClearFlag_NE(ctx->uart);
		LL_USART_ClearFlag_ORE(ctx->uart);
		LL_USART_ClearFlag_IDLE(ctx->uart);

		LL_DMA_SetMemoryAddress(ctx->dma_rx, ctx->dma_rx_channel, (uint32_t)data);
		LL_DMA_SetDataLength(ctx->dma_rx, ctx->dma_rx_channel, maxSize);

		DMA_ClearFlag_TC(ctx->dma_rx, ctx->dma_rx_channel);

		LL_USART_EnableDMAReq_RX(ctx->uart); // Включаем DMA запросы от UART 
		LL_DMA_EnableChannel(ctx->dma_rx, ctx->dma_rx_channel);
		
		LL_USART_EnableIT_IDLE(ctx->uart); // Разрешаем прерывание IDLE

		*rxLen = ModbusRTU_dmaRxCplt_wait(ctx); // Ждем окончания приема
	}
}



void ModbusRTU_dmaTxCpltCallback(ModbusRTU_t *ctx, BaseType_t *pxHigherPriorityTaskWoken)
{
	assert(ctx);
	if(ctx->dma_tx == NULL) return;
	if(DMA_IsActiveFlag_TC(ctx->dma_tx, ctx->dma_tx_channel))
	{
		DMA_ClearFlag_TC(ctx->dma_tx, ctx->dma_tx_channel);

		LL_USART_DisableDMAReq_TX(ctx->uart); // Отключаем DMA-req у UART
		LL_DMA_DisableChannel(ctx->dma_tx, ctx->dma_tx_channel); // отключаем канал DMA
		xSemaphoreGiveFromISR(ctx->txDoneSem, pxHigherPriorityTaskWoken);
	}
}

void ModbusRTU_IDLECallback(ModbusRTU_t *ctx, BaseType_t *pxHigherPriorityTaskWoken)
{
	assert(ctx);
	if(ctx->uart == NULL) return;
	if(LL_USART_IsActiveFlag_IDLE(ctx->uart))
    {
      LL_USART_ClearFlag_IDLE(ctx->uart);

			/* Отключаем DMA запрос UART */
			LL_USART_DisableDMAReq_RX(ctx->uart);

			if(ctx->dma_rx){
				LL_DMA_DisableChannel(ctx->dma_rx, ctx->dma_rx_channel);  //  Останавливаем DMA

				// Сколько байт принято = изначальная длина - оставшийся счетчик
				uint16_t received = ctx->rx_buffer_size - LL_DMA_GetDataLength(ctx->dma_rx, ctx->dma_rx_channel);
				ctx->rx_len = received; //  Запоминаем размер принятого пакета
			}
			xSemaphoreGiveFromISR(ctx->rxDoneSem, pxHigherPriorityTaskWoken);
	}
}





