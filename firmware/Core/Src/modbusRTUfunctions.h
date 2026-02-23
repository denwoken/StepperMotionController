#pragma once

#include "slave.h"
#include "DeviceRegisters_map.h"
#include "DeviceRegistersMapper.h"
#include "FreeRTOS.h"

static inline uint16_t modbus_abs_addr(uint16_t index)
{
	// Accept absolute Modbus addresses (40000+) or 0-based indices.
	if (index >= REG_BASE_ADDRESS) return index;
	const uint32_t abs = (uint32_t)REG_BASE_ADDRESS + (uint32_t)index;
	if (abs > 0xFFFFu) return 0xFFFFu;
	return (uint16_t)abs;
}

static inline bool req_covers_words(uint16_t abs_start, uint16_t count, uint16_t reg_base, uint8_t words)
{
	const uint16_t req_end = (uint16_t)(abs_start + count - 1u);
	const uint16_t reg_end = (uint16_t)(reg_base + words - 1u);
	return (abs_start <= reg_base) && (req_end >= reg_end);
}


static LIGHTMODBUS_RET_ERROR _modbusParseRequest03(
	ModbusSlave *status,
	uint8_t function,
	const uint8_t *requestPDU,
	uint8_t requestLength)
{
	// Check frame length
	if (requestLength != 5)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	const uint16_t maxCount = 125;
	uint16_t index = modbusRBE(&requestPDU[1]);
	uint16_t count = modbusRBE(&requestPDU[3]);

	// Check count
	if (count == 0 || count > maxCount)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// Address range check (0-based index or absolute Modbus address)
	if (modbusCheckRangeU16(index, count))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	const uint16_t abs_start = modbus_abs_addr(index);
	if (!reg_address_check_range(abs_start, count))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	// Validate all addresses first (no partial responses)
	for (uint16_t i = 0; i < count; i++) {
		reg_lut_t lut = reg_id_from_address((uint16_t)(abs_start + i));
		if (lut.reg_id == REG_ID__NONE)
			return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
		if(lut.reg_id == REG_ID__RESERVED) continue;
		const reg_meta_t* meta = get_reg_meta(lut.reg_id);
		if (!meta)
			return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
	}

	// Snapshot for consistent read
	deviceRegsSnapshotCapture();

	// ---- RESPONSE ----
	uint8_t dataLength = (uint8_t)(count << 1);
	if (modbusSlaveAllocateResponse(status, 2 + dataLength)) {
		deviceRegsSnapshotInvalidate();
		return MODBUS_GENERAL_ERROR(ALLOC);
	}

	status->response.pdu[0] = function;
	status->response.pdu[1] = dataLength;

	for (uint16_t i = 0; i < count; i++) {
		const uint16_t addr = (uint16_t)(abs_start + i);
		reg_lut_t lut = reg_id_from_address(addr);
		const reg_meta_t* meta = get_reg_meta(lut.reg_id);
		uint16_t word = 0u;
		if (!meta || lut.reg_id == REG_ID__RESERVED) {
			word = 0u; // reserved/unreadable -> zero
		} else {
			wordData val = readRegisterWordData(lut.motor, meta);
			if (meta->words == 2u) {
				// Write both words at once (low, then high)
				if ((i + 1u) < count) {
					modbusWBE(&status->response.pdu[2 + (i << 1)], val.u16w[0]);
					modbusWBE(&status->response.pdu[2 + ((i + 1u) << 1)], val.u16w[1]);
				}
				i++; // consume next word
				continue;
			} else {
				word = val.u16;
			}
		}

		modbusWBE(&status->response.pdu[2 + (i << 1)], word);
	}

	deviceRegsSnapshotInvalidate();
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
	uint16_t index = modbusRBE(&requestPDU[1]);
	wordData w = {0};
	w.u16 = modbusRBE(&requestPDU[3]);

	// Address range check
	if (modbusCheckRangeU16(index, 1))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	const uint16_t abs_addr = modbus_abs_addr(index);
	if (!reg_address_check_range(abs_addr, 1))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	reg_lut_t lut = reg_id_from_address(abs_addr);
	if (lut.reg_id == REG_ID__NONE)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
		
	const reg_meta_t* meta = get_reg_meta(lut.reg_id);
	if (!meta)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	// Only whole 16-bit registers can be written with function 06
	if (meta->words != 1u)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	if ((meta->access & REG_ACC_W) == 0u)
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	// Commit write (single register)
	portENTER_CRITICAL();
	bool ok = writeRegisterData(lut.motor, meta, w);
	portEXIT_CRITICAL();
	if (!ok)	return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// ---- RESPONSE ----
	if (modbusSlaveAllocateResponse(status, 5))
		return MODBUS_GENERAL_ERROR(ALLOC);

	status->response.pdu[0] = function;
	modbusWBE(&status->response.pdu[1], index);
	modbusWBE(&status->response.pdu[3], w.u16);

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
	const uint16_t maxCount = 123;
	uint16_t index = modbusRBE(&requestPDU[1]);
	uint16_t count = modbusRBE(&requestPDU[3]);
	uint8_t declaredLength = requestPDU[5];

	// Check if the declared length is correct
	if (declaredLength == 0 || declaredLength != (uint8_t)(requestLength - 6))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// Check count
	if (count == 0 || count > maxCount || declaredLength != (uint8_t)(count << 1))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);

	// Address range check
	if (modbusCheckRangeU16(index, count))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	const uint16_t abs_start = modbus_abs_addr(index);
	if (!reg_address_check_range(abs_start, count))
		return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);

	// Validate all writes first (no partial commit)
	for (uint16_t i = 0; i < count; i++) {
		const uint16_t addr = (uint16_t)(abs_start + i);
		reg_lut_t lut = reg_id_from_address(addr);
		if (lut.reg_id == REG_ID__NONE)
			return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
		const reg_meta_t* meta = get_reg_meta(lut.reg_id);
		if (!meta)
			return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
		if ((meta->access & REG_ACC_W) == 0u)
			return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
		if (meta->words == 2u) {
			const uint16_t base = reg_address_from_id(meta->id, lut.motor);
			if (!req_covers_words(abs_start, count, base, meta->words))
				return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
		} else if (meta->words != 1u) {
			return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
		}
	}

	// Commit writes after validation
	portENTER_CRITICAL();
	for (uint16_t i = 0; i < count; i++) {
		const uint16_t addr = (uint16_t)(abs_start + i);
		reg_lut_t lut = reg_id_from_address(addr);
		const reg_meta_t* meta = get_reg_meta(lut.reg_id);
		if (!meta) {
			portEXIT_CRITICAL();
			return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
		}

		wordData w = {0};
		if (meta->words == 2u) {
			if(i+1 >= count){
				portEXIT_CRITICAL();
				return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_ADDRESS);
			}
			w.u16w[0] = modbusRBE(&requestPDU[6 + (i << 1)]);
			w.u16w[1] = modbusRBE(&requestPDU[6 + ((i + 1u) << 1)]);
			i++;
		} else {
			w.u16 = modbusRBE(&requestPDU[6 + (i << 1)]);
		}
		if (!writeRegisterData(lut.motor, meta, w)) {
			portEXIT_CRITICAL();
			return modbusBuildException(status, function, MODBUS_EXCEP_ILLEGAL_VALUE);
		}
	}
	portEXIT_CRITICAL();

	// ---- RESPONSE ----
	if (modbusSlaveAllocateResponse(status, 5))
		return MODBUS_GENERAL_ERROR(ALLOC);

	status->response.pdu[0] = function;
	modbusWBE(&status->response.pdu[1], index);
	modbusWBE(&status->response.pdu[3], count);

	return MODBUS_NO_ERROR();
}



















static ModbusSlaveFunctionHandler modbusSlaveMyFunctions[] =
{
	{3, _modbusParseRequest03},
	{6, _modbusParseRequest06},
	{16, _modbusParseRequest16},
	{0, NULL}// Guard - prevents 0 array size
};
static const uint8_t modbusSlaveMyFunctionsCount =
	sizeof(modbusSlaveMyFunctions) / sizeof(modbusSlaveMyFunctions[0]) - 1;
