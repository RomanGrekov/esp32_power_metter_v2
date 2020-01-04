#include "confd.h"
#include <Arduino.h>

Confd::Confd(EepromCli &eeprom)
: _eeprom(eeprom)
{
    //_eeprom = eeprom;
}

uint8_t Confd::read_sensors(uint8_t *all_sensors, uint8_t amount)
{
    return _eeprom.read_bytes(SENSOR_ADDR_START, all_sensors, amount);
}

uint8_t Confd::store_sensors(uint8_t *all_sensors, uint8_t amount){
    return _eeprom.write_bytes(SENSOR_ADDR_START, all_sensors, amount);
}

//uint8_t Confd::write_kwh(float kwh)
//{
//    return _eeprom.write_float(KWH_ADDRESS, kwh);
//}
