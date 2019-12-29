#include "sensors.h"

uint8_t Sensor::push_one_charging(OneCharge charge){
    if(chargings_n == MAX_CHARGINGS_AMOUNT){
        for (int id=0; id<MAX_CHARGINGS_AMOUNT-1; id++){
            chargings[id].start = chargings[id+1].start;
            chargings[id].finish = chargings[id+1].finish;
            chargings[id].start_kwh = chargings[id+1].start_kwh;
            chargings[id].finish_kwh = chargings[id+1].finish_kwh;
        }
        chargings_n = MAX_CHARGINGS_AMOUNT-1;
    }
    chargings[chargings_n].start = charge.start;
    chargings[chargings_n].finish = charge.finish;
    chargings[chargings_n].start_kwh = charge.start_kwh;
    chargings[chargings_n].finish_kwh = charge.finish_kwh;
    chargings_n++;

    return chargings_n;
};
