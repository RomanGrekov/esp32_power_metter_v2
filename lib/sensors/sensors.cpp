#include "sensors.h"

uint8_t Sensor::push_one_charging(OneCharge charge){
    if(this->chargings_n == MAX_CHARGINGS_AMOUNT){
        for (int id=0; id<MAX_CHARGINGS_AMOUNT-1; id++){
            this->chargings[id].start = this->chargings[id+1].start;
            this->chargings[id].finish = this->chargings[id+1].finish;
            this->chargings[id].start_kwh = this->chargings[id+1].start_kwh;
            this->chargings[id].finish_kwh = this->chargings[id+1].finish_kwh;
        }
        this->chargings_n = MAX_CHARGINGS_AMOUNT-1;
    }
    this->chargings[this->chargings_n].start = charge.start;
    this->chargings[this->chargings_n].finish = charge.finish;
    this->chargings[this->chargings_n].start_kwh = charge.start_kwh;
    this->chargings[this->chargings_n].finish_kwh = charge.finish_kwh;
    this->chargings_n++;

    return chargings_n;
};

bool Sensor::get_charging(void){
    return this->is_charging;
}

void Sensor::set_charging(bool charging){
    this->is_charging = charging;
}

bool Sensor::is_charging_detected(void){
    if (this->is_charging && this->is_charging != this->is_charging_old){
        this->is_charging_old = this->is_charging;
        return true;
    }
    return false;
}

bool Sensor::is_stop_charging_detected(void){
    if (!this->is_charging && this->is_charging != this->is_charging_old){
        this->is_charging_old = this->is_charging;
        return true;
    }
    return false;
}

void Sensor::set_a(float a){
    if (isnan(a)) a = 0;
    this->A = a;
}

void Sensor::set_v(float v){
    if (isnan(v)) v = 0;
    this->V = v;
}

void Sensor::set_kwh(float kwh){
    if (isnan(kwh)) kwh = 0;
    this->Kwh = kwh;
}

float Sensor::get_v(void){
    return this->V;
}

float Sensor::get_a(void){
    return this->A;
}

float Sensor::get_kwh(void){
    return this->Kwh;
}

bool Sensor::is_data_changed(void){
    bool changed = false;
    //Serial.printf("%d: %f - %f\n", this->address, this->A, this->A_old);
    if (this->V != this->V_old || this->A != this->A_old || this->Kwh != this->Kwh_old){
        changed = true;
        this->A_old = this->A;
        this->V_old = this->V;
        this->Kwh_old = this->Kwh;
    }
    return changed;
}

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
