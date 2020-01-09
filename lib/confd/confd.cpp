#include "confd.h"
#include <Arduino.h>

Confd::Confd(EepromCli &eeprom)
: _eeprom(eeprom)
{
    //_eeprom = eeprom;
}

uint8_t Confd::read_sensors(uint8_t *all_sensors) {
    return _eeprom.read_bytes(SENSOR_ADDR_START, all_sensors, SENSOR_AMOUNT);
}
uint8_t Confd::store_sensors(uint8_t *all_sensors){
    return _eeprom.write_bytes(SENSOR_ADDR_START, all_sensors, SENSOR_AMOUNT);
}

uint8_t Confd::read_wifi_name(uint8_t *name) {
    return _eeprom.read_bytes(WIFI_NAME_ADDR_START, name, WIFI_NAME_ADDR_SIZE);
}
uint8_t Confd::store_wifi_name(uint8_t *name){
    return _eeprom.write_bytes(WIFI_NAME_ADDR_START, name, WIFI_NAME_ADDR_SIZE);
}

uint8_t Confd::read_wifi_pw(uint8_t *pw) {
    return _eeprom.read_bytes(WIFI_PW_ADDR_START, pw, WIFI_PW_ADDR_SIZE);
}
uint8_t Confd::store_wifi_pw(uint8_t *pw){
    return _eeprom.write_bytes(WIFI_PW_ADDR_START, pw, WIFI_PW_ADDR_SIZE);
}


//uint8_t Confd::write_kwh(float kwh)
//{
//    return _eeprom.write_float(KWH_ADDRESS, kwh);
//}
