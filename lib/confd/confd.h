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

#define SENSORS_AMOUNT 10
#define DUMMY_INDEX 255


class Confd
{
public:
    Confd(EepromCli &eeprom);
    uint8_t read_sensors(void);
    uint8_t get_sensors_n(void);
    uint8_t get_sensors_indexes(uint8_t *indexes);
    uint8_t get_address(uint8_t index);
    void add_sensor(uint8_t indes, uint8_t addr);
    void del_sensor(uint8_t index);
    uint8_t store_sensors(void);

    uint8_t sensors[SENSORS_AMOUNT];
    uint8_t sensors_indexes[SENSORS_AMOUNT];
    uint8_t cur_index;
private:
    void update_indexes(void);
    void add_index(uint8_t index);
    void del_index(uint8_t index);
    void clean_indexes(void);

    EepromCli& _eeprom;

};

#endif
