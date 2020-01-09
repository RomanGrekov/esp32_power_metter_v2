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
#define WIFI_NAME_ADDR_SIZE 16
#define WIFI_PW_ADDR_START (WIFI_NAME_ADDR_START + WIFI_NAME_ADDR_SIZE + 1)
#define WIFI_PW_ADDR_SIZE 16

enum WifiModeEnum {
    sta_mode = 1, // Connect to router
    ap_mode       // Create access point
};
union WifiMode {
    WifiModeEnum mode;
    uint8_t mode_array[4];
};

#define WIFI_MODE_ADDR_START (WIFI_PW_ADDR_START + WIFI_PW_ADDR_SIZE + 1)
#define WIFI_MODE_ADDR_SIZE 4

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
    uint8_t read_wifi_mode(WifiMode *mode);
    uint8_t store_wifi_mode(WifiMode *mode);
private:
    EepromCli& _eeprom;

};

#endif
