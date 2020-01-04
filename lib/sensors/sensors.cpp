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

uint8_t get_sensors_indexes(uint8_t *all_sensors, uint8_t *indexes, uint8_t amount){
    uint8_t addr=0;
    uint8_t cur_index=0;

    // Search non zero address
    for(int i=0; i<amount; i++){
        addr = all_sensors[i];
        if (addr != 0){
            indexes[cur_index] = i;
            cur_index++;
        }
    }
    return cur_index;
}
