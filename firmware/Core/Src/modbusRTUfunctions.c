


#include "slave.h"
#include "DeviceRegisters_map.h"


static LIGHTMODBUS_RET_ERROR _modbusParseRequest03(
	ModbusSlave *status,
	uint8_t function,
	const uint8_t *requestPDU,
	uint8_t requestLength)
{
	// Check frame length
	if (requestLength != 5)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	const uint16_t maxCount =  125;

	uint16_t index = modbusRBE(&requestPDU[1]);
	uint16_t count = modbusRBE(&requestPDU[3]);

	// Check count
	if (count == 0 || count > maxCount)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// Addresss range check
	if (modbusCheckRangeU16(index, count))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	// Prepare callback args
	//ModbusRegisterCallbackResult cres;

	// Check if all registers can be read
	// for (uint16_t i = 0; i < count; i++)
	// {
	// 	reg_lut_t lut = reg_id_from_address(index + i);
	// 	if(lut.reg_id == REG_ID__NONE)
	// 		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS)	
	// }
	if(!reg_address_range_isvalid(REG_BASE_ADDRESS+index,count))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS)	


	// ---- RESPONSE ----

	uint8_t dataLength = (count << 1);
	if (modbusSlaveAllocateResponse(status, 2 + dataLength))
		return MODBUS_GENERAL_ERROR(ALLOC);

	status->response.pdu[0] = function;
	status->response.pdu[1] = dataLength;
	
	//memset(&status->response.pdu[2],0,dataLength);

	for (uint16_t i = 0; i < count; i++)
	{
		reg_lut_t lut = reg_id_from_address(index + i);
		if(lut.reg_id == REG_ID__NONE)
			return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS)	
		reg_meta_t* meta = get_reg_meta(lut.reg_id);

		uint32_t val = takeRegisterData(lut.motor, meta);
		

		
		modbusWBE(&status->response.pdu[2 + (i << 1)], val);
		
	}

	return MODBUS_NO_ERROR();
}





static LIGHTMODBUS_RET_ERROR _modbusParseRequest06(
	ModbusSlave *status,
	uint8_t function,
	const uint8_t *requestPDU,
	uint8_t requestLength)
{
	// Check frame length
	if (requestLength != 5)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// Get register index and value
	ModbusDataType datatype = function == 5 ? MODBUS_COIL : MODBUS_HOLDING_REGISTER;
	uint16_t index = modbusRBE(&requestPDU[1]);
	uint16_t value = modbusRBE(&requestPDU[3]);

	// For coils - check if coil value is valid
	if (datatype == MODBUS_COIL && value != 0x0000 && value != 0xFF00)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// Prepare callback args
	ModbusRegisterCallbackResult cres;
	ModbusRegisterCallbackArgs cargs = {
		.type = datatype,
		.query = MODBUS_REGQ_W_CHECK,
		.index = index,
		.value = (uint16_t)((datatype == MODBUS_COIL) ? (value != 0) : value),
		.function = function,
	};

	// Check if the register/coil can be written
	ModbusError fail = status->registerCallback(status, &cargs, &cres);
	if (fail) return modbusBuildException(status, function, MODBUS_EXCEP_SLAVE_FAILURE);
	if (cres.exceptionCode) return modbusBuildException(status, function, cres.exceptionCode);

	// Write coil/register
	// Keep in mind that 0xff00 is 0 when cast to uint8_t
	cargs.query = MODBUS_REGQ_W;
	(void) status->registerCallback(status, &cargs, &cres);

	// ---- RESPONSE ----

	if (modbusSlaveAllocateResponse(status, 5))
		return MODBUS_GENERAL_ERROR(ALLOC);

	status->response.pdu[0] = function;
	modbusWBE(&status->response.pdu[1], index);
	modbusWBE(&status->response.pdu[3], value);

	return MODBUS_NO_ERROR();
}


static LIGHTMODBUS_RET_ERROR _modbusParseRequest16(
	ModbusSlave *status,
	uint8_t function,
	const uint8_t *requestPDU,
	uint8_t requestLength)
{
	// Check length
	if (requestLength < 6)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// Get first index and register count
	ModbusDataType datatype = function == 15 ? MODBUS_COIL : MODBUS_HOLDING_REGISTER;
	uint16_t maxCount = datatype == MODBUS_COIL ? 1968 : 123;
	uint16_t index = modbusRBE(&requestPDU[1]);
	uint16_t count = modbusRBE(&requestPDU[3]);
	uint8_t declaredLength = requestPDU[5];

	// Check if the declared length is correct
	if (declaredLength == 0 || declaredLength != requestLength - 6)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// Check count
	if (count == 0
		|| count > maxCount
		|| declaredLength != (datatype == MODBUS_COIL ? modbusBitsToBytes(count) : (count << 1)))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// Addresss range check
	if (modbusCheckRangeU16(index, count))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	// Prepare callback args
	ModbusRegisterCallbackResult cres;
	ModbusRegisterCallbackArgs cargs = {
		.type = datatype,
		.query = MODBUS_REGQ_W_CHECK,
		.index = 0,
		.value = 0,
		.function = function,
	};

	// Check write access
	for (uint16_t i = 0; i < count; i++)
	{
		cargs.index = index + i;
		cargs.value = datatype == MODBUS_COIL ? modbusMaskRead(&requestPDU[6], i) : modbusRBE(&requestPDU[6 + (i << 1)]);
		ModbusError fail = status->registerCallback(status, &cargs, &cres);
		if (fail) return modbusBuildException(status, function, MODBUS_EXCEP_SLAVE_FAILURE);
		if (cres.exceptionCode) return modbusBuildException(status, function, cres.exceptionCode);
	}

	// Write coils
	cargs.query = MODBUS_REGQ_W;
	for (uint16_t i = 0; i < count; i++)
	{
		cargs.index = index + i;
		cargs.value = datatype == MODBUS_COIL ? modbusMaskRead(&requestPDU[6], i) : modbusRBE(&requestPDU[6 + (i << 1)]);
		(void) status->registerCallback(status, &cargs, &cres);
	}

	// ---- RESPONSE ----

	if (modbusSlaveAllocateResponse(status, 5))
		return MODBUS_GENERAL_ERROR(ALLOC);

	status->response.pdu[0] = function;
	modbusWBE(&status->response.pdu[1], index);
	modbusWBE(&status->response.pdu[3], count);	

	return MODBUS_NO_ERROR();
}



















ModbusSlaveFunctionHandler modbusSlaveDefaultFunctions[] =
{
	{3, _modbusParseRequest03},
	{6, _modbusParseRequest06},
	{16, _modbusParseRequest16},
	{0, NULL}// Guard - prevents 0 array size
};