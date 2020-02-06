#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "PZEM004Tv30.h"


/*
  Struct with sensors data
*/
#define MAX_CHARGINGS_AMOUNT 5

struct OneCharge {
    long int start=0;
    long int finish=0;
    float start_kwh=0.0;
    float finish_kwh=0.0;
};

class Sensor {
public:
  uint8_t address=0;

  uint8_t push_one_charging(OneCharge charge);
  void set_v(float v);
  void set_a(float a);
  void set_kwh(float kwh);
  float get_v(void);
  float get_a(void);
  float get_kwh(void);
  bool is_data_changed(void);
  bool get_charging(void);
  void set_charging(bool chargeing);
  bool is_charging_detected(void);
  bool is_stop_charging_detected(void);

private:
  OneCharge chargings[MAX_CHARGINGS_AMOUNT];
  uint8_t chargings_n=0;
  bool is_charging;
  bool is_charging_old;

  float V=0;
  float A=0;
  float Kwh=0;
  float V_old=0;
  float A_old=0;
  float Kwh_old=0;

};

uint8_t get_sensors_indexes(uint8_t *all_sensors, uint8_t *indexes, uint8_t amount);

#endif
