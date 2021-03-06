#ifndef LEDS_H
#define LEDS_H

#include <Adafruit_MCP23017.h>
#include "FreeRTOS.h"
#include <Arduino.h>

/*
    Port expander
*/

class McpLeds
{
public:
    McpLeds(int address, uint8_t leds_n);
    void init();
    void off(uint8_t led_n);
    void off_all(void);
    void on(uint8_t led_n);
    void on_only(uint8_t led_n);

    void define_mutex(SemaphoreHandle_t mutex);

private:
    void set(uint8_t led_n, uint8_t state);

    Adafruit_MCP23017 mcp;
    uint8_t _leds_n;
    int _address;

    SemaphoreHandle_t xMutexI2c;
};

#endif
