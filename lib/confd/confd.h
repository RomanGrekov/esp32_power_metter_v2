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

#define SENSOR_ADDR_START (START_ADDRESS + 10)
#define SENSOR_AMOUNT 10

#define WIFI_NAME_ADDR_START (SENSOR_ADDR_START + SENSOR_AMOUNT + 1)
#define WIFI_NAME_ADDR_SIZE 8
#define WIFI_PW_ADDR_START (WIFI_NAME_ADDR_START + WIFI_NAME_ADDR_SIZE + 1)
#define WIFI_PW_ADDR_SIZE 8

class Confd
{
public:
    Confd(EepromCli &eeprom);
    uint8_t read_sensors(uint8_t *all_sensorst);
    uint8_t store_sensors(uint8_t *all_sensorst);

    uint8_t read_wifi_name(uint8_t *name);
    uint8_t store_wifi_name(uint8_t *name);
    uint8_t read_wifi_pw(uint8_t *pw);
    uint8_t store_wifi_pw(uint8_t *pw);
private:
    EepromCli& _eeprom;

};

#endif
