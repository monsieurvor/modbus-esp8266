/*
    Modbus Library for Arduino
    ModbusTCP for W5x00 Ethernet
    Copyright (C) 2022 Alexander Emelianov (a.m.emelianov@gmail.com)
*/

#pragma once
#if defined(MODBUSIP_USE_DNS)
#include <Dns.h>
#endif
#include "ModbusAPI.h"
#include "ModbusTCPTemplate.h"
#if defined(ARDUINO_PORTENTA_H7_M4) || defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_PORTENTA_X8)
#define MODBUS_ETH_WRAP_ACCEPT
#undef MODBUS_ETH_WRAP_BEGIN
#elif defined(ESP32)
#undef MODBUS_ETH_WRAP_ACCEPT
#define MODBUS_ETH_WRAP_BEGIN
#else
#undef MODBUS_ETH_WRAP_ACCEPT
#undef MODBUS_ETH_WRAP_BEGIN
#endif

class ModbusEthernet : public ModbusAPI<ModbusTCPTemplate<EthernetClient>> {
#if defined(MODBUSIP_USE_DNS)
    private:
    static IPAddress resolver (const char* host) {
        DNSClient dns;
        IPAddress ip;
        
        dns.begin(Ethernet.dnsServerIP());
        if (dns.getHostByName(host, ip) == 1)
            return ip;
        else
            return IPADDR_NONE;
    }
    public:
    ModbusEthernet() : ModbusAPI() {
        resolve = resolver;
    }
#endif
};
