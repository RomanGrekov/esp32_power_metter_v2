#include "eeprom_cli.h"


//EepromCli::EepromCli(uint8_t sda, uint8_t scl, uint16_t address)
EepromCli::EepromCli(uint16_t address)
{
    //_sda = sda;
    //_scl = scl;
    //_wire.begin(sda, scl);
    //_wire.begin();
    Wire.begin();
    _eeprom_address = address;
}

void EepromCli::define_mutex(SemaphoreHandle_t mutex){
    xMutexI2c = mutex;
}

void EepromCli::force_begin(void){
    //_wire.begin(_sda, _scl);
    Wire.begin();
}

void EepromCli::_set_addr(uint16_t address)
{
    //_wire.write((uint8_t)(address >> 8));
    Wire.write((uint8_t)(address >> 8));
    //_wire.write((uint8_t)(address & 0b0000000011111111));
    Wire.write((uint8_t)(address & 0b0000000011111111));
}

uint8_t EepromCli::read_byte(uint16_t addr, uint8_t *dest)
{
    uint8_t error;
    uint8_t data;

    if (xMutexI2c == NULL || xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        //_wire.beginTransmission(_eeprom_address);
        Wire.beginTransmission(_eeprom_address);
        _set_addr(addr);
        //error = _wire.endTransmission();
        error = Wire.endTransmission();
        if (error != 0){
            if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
            return error;
        }
        //_wire.requestFrom(_eeprom_address, 1);
        Wire.requestFrom(_eeprom_address, 1);
        //if (_wire.available()){
        if (Wire.available()){
            //data = _wire.read();
            data = Wire.read();
            *dest = data;
        }
        else{
            if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
            return 10;
        }
        if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
    }
    return 0;
}

uint8_t EepromCli::write_byte(uint16_t addr, uint8_t data)
{
    uint8_t error;

    if (xMutexI2c == NULL || xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        //_wire.beginTransmission(_eeprom_address);
        Wire.beginTransmission(_eeprom_address);
        _set_addr(addr);
        //_wire.write(data);
        Wire.write(data);
        vTaskDelay(pdMS_TO_TICKS(5));
        //error = _wire.endTransmission();
        error = Wire.endTransmission();
        vTaskDelay(pdMS_TO_TICKS(5));
        if (error != 0){
            if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
            return error;
        }
        if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
    }
    return 0;
}

uint8_t EepromCli::read_bytes(uint16_t addr, uint8_t *dest, uint16_t size)
{
    uint8_t error;
    uint16_t _addr;
    for (uint16_t i=0; i<size; i++){
        _addr = addr + i;
        error = read_byte(_addr, dest+i);
        if (error != 0){
            return error;
        }
    }
    return 0;
}

uint8_t EepromCli::write_bytes(uint16_t addr, uint8_t *source, uint16_t size)
{
    uint8_t error;
    uint16_t _addr;
    for (uint16_t i=0; i<size; i++){
        _addr = addr + i;
        error = write_byte(_addr, *(source+i));
        if (error != 0){
            return error;
        }
    }
    return 0;
}

uint8_t EepromCli::read_float(uint16_t addr, float *f)
{
    union {
        float _f;
        uint8_t _byte[4];
    } u;
    uint8_t error;

    error = read_bytes(addr, u._byte, sizeof(u._byte));
    *f = u._f;
    return error;
}

uint8_t EepromCli::write_float(uint16_t addr, float f)
{
    union {
        float _f;
        uint8_t _byte[4];
    } u;
    u._f = f;
    return write_bytes(addr, u._byte, sizeof(u._byte));
}

/*
uint8_t Confd::write_kwh(float kwh)
{
    uint8_t error;
    uint16_t addr;
    union Float f;
    f._float = kwh;
    _log.trace("Confd: Write Kwh..."CR);
    for (uint16_t i=0; i<sizeof(f._byte); i++){
        addr = KWH_ADDRESS + i;
        error = _eeprom.write_byte(addr, *(f._byte+i));
        if (error != 0){
            _log.error("Failed to write KWh"CR);
            return 1;
        }
    }
    _log.trace(" Ok"CR);

    return 0;
}
*/

/*
uint8_t EepromCli::read_key(uint8_t *key)
{
    uint8_t error;
    uint16_t addr;

    _log.notice("EEPROM_CLI: Read key...");
    for (uint16_t i=0; i<KEY_SIZE; i++){
        addr = KEY_ADDR + i;
        error = read_byte(addr, key+i);
        if (error != 0){
            _log.error(" Failed"CR);
            return 1;
        }
    }
    _log.notice(" Ok"CR);

    return 0;
}

uint8_t EepromCli::write_key(uint8_t *key)
{
    uint8_t error;
    uint16_t addr;

    _log.notice("EEPROM_CLI: Write key...");
    for (uint16_t i=0; i<KEY_SIZE; i++){
        addr = KEY_ADDR + i;
        error = write_byte(addr, *(key + i));
        if (error != 0){
            _log.error(" Failed"CR);
            return 1;
        }
    }
    _log.notice(" Ok"CR);

    return 0;
}

uint8_t EepromCli::is_key_saved(bool *result)
{
    uint8_t error;
    uint8_t res;

    _log.notice("EEPROM_CLI: Is key saved...");
    error = read_byte(KEY_SAVED_FLAF_ADDR, &res);
    if (error != 0){
        _log.error(" Failed"CR);
        return 1;
    }
    _log.notice(" Ok"CR);

    if (res != KEY_SAVED_FLAG_VAL) *result = false;
    else *result = true;
    return 0;
}

uint8_t EepromCli::set_key_saved(void)
{
    uint8_t error;

    _log.notice("EEPROM_CLI: Set key saved...");
    error = write_byte(KEY_SAVED_FLAF_ADDR, KEY_SAVED_FLAG_VAL);
    if (error != 0){
        _log.error(" Failed"CR);
        return 1;
    }
    _log.notice(" Ok"CR);
    return 0;
}

uint8_t EepromCli::set_key_unsaved(void)
{
    uint8_t error;

    _log.notice("EEPROM_CLI: Set key unsaved...");
    error = write_byte(KEY_SAVED_FLAF_ADDR, 0xFF);
    if (error != 0){
        _log.error(" Failed"CR);
        return 1;
    }
    _log.notice(" Ok"CR);
}
*/
