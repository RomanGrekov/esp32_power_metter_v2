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

uint8_t Confd::get_working_sensors_n(){
    uint8_t addr=0;
    uint8_t sens_amount=0;
    // Search non zero address
    for(int i=0; i<SENSORS_AMOUNT-1; i++){
        addr = get_address(i);
        if (addr != 0){
            sens_amount++;
        }
    }
    return sens_amount;
}
//uint8_t Confd::write_kwh(float kwh)
//{
//    return _eeprom.write_float(KWH_ADDRESS, kwh);
//}
