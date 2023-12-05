/*
    Modbus Library for Arduino
	Modbus public API implementation
    Copyright (C) 2014 Andr� Sarmento Barbosa
                  2017-2021 Alexander Emelianov (a.m.emelianov@gmail.com)
*/
#pragma once
#include "Modbus.h"

template <class T>
class ModbusAPI : public T {
	public:
	template <typename TYPEID>
	uint16_t writeHreg(TYPEID id, uint16_t offset, uint16_t value, cbTransaction cb = nullptr, uint16_t port = 0, uint8_t unit = MODBUSIP_UNIT);
	template <typename TYPEID>
	uint16_t writeHreg(TYPEID id, uint16_t offset, uint16_t* value, uint16_t numregs = 1, cbTransaction cb = nullptr, uint16_t port = 0, uint8_t unit = MODBUSIP_UNIT);
	template <typename TYPEID>
	uint16_t readHreg(TYPEID id, uint16_t offset, uint16_t* value, uint16_t numregs = 1, cbTransaction cb = nullptr, uint16_t port = 0, uint8_t unit = MODBUSIP_UNIT);
	template <typename TYPEID>
	uint16_t readIreg(TYPEID id, uint16_t offset, uint16_t* value, uint16_t numregs = 1, cbTransaction cb = nullptr, uint16_t port = 0, uint8_t unit = MODBUSIP_UNIT);

	template <typename TYPEID>
	uint16_t rawRequest(TYPEID ip, uint8_t* data, uint16_t len, cbTransaction cb = nullptr, uint16_t port = 0, uint8_t unit = MODBUSIP_UNIT);
	template <typename TYPEID>
	uint16_t rawResponce(TYPEID ip, uint8_t* data, uint16_t len, uint16_t port = 0, uint8_t unit = MODBUSIP_UNIT);
	template <typename TYPEID>
	uint16_t errorResponce(TYPEID ip, Modbus::FunctionCode fn, Modbus::ResultCode excode, uint16_t port = 0, uint8_t unit = MODBUSIP_UNIT);
};

// FNAME	writeHreg, writeIreg
// REG		HREG, IREG
// FUNC		Modbus function
// MAXNUM	Register count limit
// VALTYPE	bool, uint16_t
// VALUE	
#define IMPLEMENT_WRITEREG(FNAME, REG, FUNC, VALUE, VALTYPE) \
template <class T> \
template <typename TYPEID> \
uint16_t ModbusAPI<T>::FNAME(TYPEID ip, uint16_t offset, VALTYPE value, cbTransaction cb, uint16_t port, uint8_t unit) { \
	this->readSlave(offset, VALUE(value), Modbus::FUNC); \
	return this->send(ip, REG(offset), cb, port, unit); \
}
IMPLEMENT_WRITEREG(writeHreg, HREG, FC_WRITE_REG, , uint16_t)

#define IMPLEMENT_WRITEREGS(FNAME, REG, FUNC, VALUE, MAXNUM, VALTYPE) \
template <class T> \
template <typename TYPEID> \
uint16_t ModbusAPI<T>::FNAME(TYPEID ip, uint16_t offset, VALTYPE* value, uint16_t numregs, cbTransaction cb, uint16_t port, uint8_t unit) { \
	if (numregs < 0x0001 || numregs > MAXNUM) return false; \
	this->VALUE(REG(offset), offset, numregs, Modbus::FUNC, value); \
	return this->send(ip, REG(offset), cb, port, unit); \
}
IMPLEMENT_WRITEREGS(writeHreg, HREG, FC_WRITE_REGS, writeSlaveWords, MODBUS_MAX_WORDS, uint16_t)

#define IMPLEMENT_READREGS(FNAME, REG, FUNC, MAXNUM, VALTYPE) \
template <class T> \
template <typename TYPEID> \
uint16_t ModbusAPI<T>::FNAME(TYPEID ip, uint16_t offset, VALTYPE* value, uint16_t numregs, cbTransaction cb, uint16_t port, uint8_t unit) { \
	if (numregs < 0x0001 || numregs > MAXNUM) return false; \
	this->readSlave(offset, numregs, Modbus::FUNC); \
	return this->send(ip, REG(offset), cb, port, unit, (uint8_t*)value); \
}
IMPLEMENT_READREGS(readHreg, HREG, FC_READ_REGS, MODBUS_MAX_WORDS, uint16_t)
IMPLEMENT_READREGS(readIreg, IREG, FC_READ_INPUT_REGS, MODBUS_MAX_WORDS, uint16_t)

template <class T>
template <typename TYPEID>
uint16_t ModbusAPI<T>::rawRequest(TYPEID ip, \
			uint8_t* data, uint16_t len,
			cbTransaction cb, uint16_t port, uint8_t unit) {
	free(this->_frame);
	this->_frame = (uint8_t*)malloc(len);
	if (!this->_frame)
		return 0;
	this->_len = len;
	memcpy(this->_frame, data, len);
    return this->send(ip, NULLREG, cb, port, unit);
};

template <class T>
template <typename TYPEID>
uint16_t ModbusAPI<T>::rawResponce(TYPEID ip, \
			uint8_t* data, uint16_t len, uint16_t port, uint8_t unit) {
	free(this->_frame);
	this->_frame = (uint8_t*)malloc(len);
	if (!this->_frame)
		return 0;
	this->_len = len;
	memcpy(this->_frame, data, len);
    return this->send(ip, NULLREG, nullptr, unit, nullptr, port, false);
};

template <class T>
template <typename TYPEID>
uint16_t ModbusAPI<T>::errorResponce(TYPEID ip, Modbus::FunctionCode fn, Modbus::ResultCode excode, uint16_t port, uint8_t unit) {
	this->exceptionResponse(fn, excode);
	return this->send(ip, NULLREG, nullptr, unit, nullptr, port, false);
}
