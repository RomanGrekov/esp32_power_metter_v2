#include "confd.h"

Confd::Confd(EepromCli &eeprom)
: _eeprom(eeprom)
{
    //_eeprom = eeprom;
}

uint8_t Confd::read_sensors()
{
    return _eeprom.read_bytes(SENSOR_ADDR_START, this->sensors, SENSORS_AMOUNT);
}

uint8_t Confd::get_address(uint8_t index){
    return this->sensors[index];
}

void Confd::add_sensor(uint8_t index, uint8_t addr){
    this->sensors[index] = addr;
}

uint8_t Confd::store_sensors(){
    return _eeprom.write_bytes(SENSOR_ADDR_START, this->sensors, SENSORS_AMOUNT);
}

//uint8_t Confd::write_kwh(float kwh)
//{
//    return _eeprom.write_float(KWH_ADDRESS, kwh);
//}
