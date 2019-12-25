#ifndef LEDS_H
#define LEDS_H

#include <Adafruit_MCP23017.h>

/*
    Port expander
*/

class McpLeds
{
public:
  McpLeds(int address, int leds_n);
  void init();
  void off(uint8_t led_n);
  void off_all(void);
  void on(uint8_t led_n);
  void on_only(uint8_t led_n);

private:
  void set(uint8_t led_n, int state);

  Adafruit_MCP23017 mcp;
  int _leds_n;
  int _address;

};

#endif
