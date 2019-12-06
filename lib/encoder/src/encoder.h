#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define CLOCKWISE_DIRECTION 0b01001011               // Clockwise direction
#define COUNTER_CLOCKWISE_DIRECTION 0b10000111       // Counter-Clockwise direction

enum Direction {
    none = 0,
    cw,
    ccw
};

class Encoder {
public:
    Encoder(uint8_t A_pin, uint8_t B_pin);
    bool isChanged();
    uint8_t getDirection();
    void setRotation(int min, int max, bool do_circle=true);
    int getRotation();
    void begin(void (*ISR_callback)(void), bool internal_pullup=false);

    void IRAM_ATTR EncChanged_ISR();

    Direction direction;
private:
    /*
    What is IRAM_ATTR?
    By flagging a piece of code with the IRAM_ATTR attribute we are declaring that the compiled code will
    be placed in the Internal RAM (IRAM) of the ESP32.
    Otherwise the code is placed in the Flash. And flash on the ESP32 is much slower than internal RAM.
    If the code we want to run is an interrupt service routine (ISR), we generally want to execute it as
    quickly as possible. If we had to ‘wait’ for an ISR to load from flash, things would go horribly wrong.
    */
	void(*ISR_callback)();
    void calc_rotation(Direction direction);

	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    uint8_t _a_pin;
    uint8_t _b_pin;

    uint8_t buffer;
    bool changed;

    int rotate_min=0;
    int rotate_max=10;
    bool rotate_cirtcle=true;
    int rotate_cur=0;

};

#endif
