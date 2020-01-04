#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <ESPDateTime.h>

/*
  Struct with sensors data
*/
#define MAX_CHARGINGS_AMOUNT 5

struct OneCharge {
    time_t start=0;
    time_t finish=0;
    float start_kwh=0.0;
    float finish_kwh=0.0;
};

class Sensor {
public:
  uint8_t address=0;
  float V;
  float A;
  float Kwh;
  bool is_charging;

  uint8_t push_one_charging(OneCharge charge);
private:
  OneCharge chargings[MAX_CHARGINGS_AMOUNT];
  uint8_t chargings_n=0;

};

uint8_t get_sensors_indexes(uint8_t *all_sensors, uint8_t *indexes, uint8_t amount);

#endif
