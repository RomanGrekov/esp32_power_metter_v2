#ifndef CONFD_H
#define CONFD_H

#include <eeprom_cli.h>
/*
#define KEY_SAVED_FLAF_ADDR (KEY_ADDR + KEY_SIZE)
#define KEY_SAVED_FLAG_VAL 0xAA
#define FIRST_RUN_ADDR (KEY_SAVED_FLAF_ADDR + 1)
#define FIRST_RUN_VAL 0xBB
*/
#define START_ADDRESS 0x0000

#define SENSOR_ADDR_SIZE 1               // 1 Byte
#define SENSOR_ADDR_START (START_ADDRESS + 10)

#define DUMMY_INDEX 255


class Confd
{
public:
    Confd(EepromCli &eeprom);
    uint8_t read_sensors(uint8_t *all_sensors, uint8_t amount);
    uint8_t store_sensors(uint8_t *all_sensors, uint8_t amount);

private:
    EepromCli& _eeprom;

};

#endif
