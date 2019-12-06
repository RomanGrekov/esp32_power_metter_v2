#include "encoder.h"

Encoder::Encoder(uint8_t A_pin, uint8_t B_pin){
    this->_a_pin = A_pin;
    this->_b_pin = B_pin;
}

void Encoder::begin(void (*ISR_callback)(void), bool internal_pullup){
    uint8_t mode = internal_pullup ? INPUT_PULLUP : INPUT;
    pinMode(this->_a_pin, mode);
    pinMode(this->_b_pin, mode);

	attachInterrupt(this->_a_pin, ISR_callback, CHANGE);
    attachInterrupt(this->_b_pin, ISR_callback, CHANGE);

    this->buffer=0;
    this->changed=0;
}


void IRAM_ATTR Encoder::EncChanged_ISR(){
	portENTER_CRITICAL_ISR(&(this->mux));
    // Save prev combination
    this->buffer <<= 2;
    // Clean yangest two bits
    this->buffer &= 0b11111100;
    // Save state of A button
    this->buffer |= (digitalRead(this->_a_pin) ? (1 << 1) : 0);
    // Save state of B button
    this->buffer |= (digitalRead(this->_b_pin) ? (1 << 0) : 0);

    if (this->buffer == CLOCKWISE_DIRECTION){
        this->direction = cw;
        this->changed = true;
        this->buffer = 0;
        calc_rotation(this->direction);
    }
    else if (this->buffer == COUNTER_CLOCKWISE_DIRECTION){
        this->direction = ccw;
        this->changed = true;
        this->buffer = 0;
        calc_rotation(this->direction);
    }
	portEXIT_CRITICAL_ISR(&(this->mux));
}

bool Encoder::isChanged(){
    return this->changed;
}

uint8_t Encoder::getDirection(){
    this->changed = 0;
    return this->direction;
}

void Encoder::calc_rotation(Direction direction){
    if (direction == cw){
        if (this->rotate_cur < this->rotate_max) this->rotate_cur++;
        else if (this->rotate_cirtcle) this->rotate_cur = this->rotate_min;
    }
    if (direction == ccw){
        if (this->rotate_cur > this->rotate_min) this->rotate_cur--;
        else if (this->rotate_cirtcle) this->rotate_cur = this->rotate_max;

    }
}

void Encoder::setRotation(int min, int max, bool do_circle){
    this->rotate_min = min;
    this->rotate_max = max;
    this->rotate_cirtcle = do_circle;
}

int Encoder::getRotation(){
    return this->rotate_cur;
}
