#include "confd.h"

Confd::Confd(EepromCli &eeprom)
: _eeprom(eeprom)
{
    //_eeprom = eeprom;
}

uint8_t Confd::read_sensors()
{
    uint8_t res;
    res = _eeprom.read_bytes(SENSOR_ADDR_START, this->sensors, SENSORS_AMOUNT);
    update_indexes();
    return res;
}

uint8_t Confd::get_sensors_n(void){
    return cur_index;
}

uint8_t Confd::get_sensors_indexes(uint8_t *indexes){
    for(int i=0; i<SENSORS_AMOUNT; i++) indexes[i] = sensors_indexes[i];
    return get_sensors_n();
}

uint8_t Confd::get_address(uint8_t index){
    return this->sensors[index];
}

void Confd::add_sensor(uint8_t index, uint8_t addr){
    this->sensors[index] = addr;
    if (addr > 0) add_index(index);
}

void Confd::del_sensor(uint8_t index){
    this->sensors[index] = 0;
    del_index(index);
}

uint8_t Confd::store_sensors(){
    return _eeprom.write_bytes(SENSOR_ADDR_START, this->sensors, SENSORS_AMOUNT);
}

void Confd::update_indexes(void){
    uint8_t addr=0;
    uint8_t sens_amount=0;

    clean_indexes();
    // Search non zero address
    for(int i=0; i<SENSORS_AMOUNT; i++){
        addr = get_address(i);
        if (addr != 0){
            add_index(i);
        }
    }
}

void Confd::add_index(uint8_t index){
    sensors_indexes[cur_index] = index;
    cur_index++;
}

void Confd::del_index(uint8_t index){
    bool start_copy=false;
    for(int i=0; i<SENSORS_AMOUNT; i++){
        if(sensors_indexes[i] == index || start_copy){
            start_copy = true;
            if(i < SENSORS_AMOUNT - 1) sensors_indexes[i] = sensors_indexes[i+1];
            else sensors_indexes[i] = DUMMY_INDEX;
            cur_index--;
        }
    }
}

void Confd::clean_indexes(void){
  for(int i=0; i<SENSORS_AMOUNT; i++) sensors_indexes[i] = DUMMY_INDEX;
  cur_index=0;
}
//uint8_t Confd::write_kwh(float kwh)
//{
//    return _eeprom.write_float(KWH_ADDRESS, kwh);
//}
