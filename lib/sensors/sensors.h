#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

class Sensors
{
public:
    void load_sensors(uint8_t *sensors, int amount);

private:
    int all_sensors_amount;
};

#endif
