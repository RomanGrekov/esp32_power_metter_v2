#include "leds.h"

McpLeds::McpLeds(int address, int leds_n)
{
  _address = address;
  _leds_n = leds_n;
}

void McpLeds::init(){
    mcp.begin(_address);
    for(int i=0; i<_leds_n; i++){
        mcp.pinMode(i, OUTPUT);
    }
}

void McpLeds::set(uint8_t led_n, int state){
    mcp.digitalWrite(led_n, state);
}

void McpLeds::off(uint8_t led_n){
    set(led_n, LOW);
}

void McpLeds::on(uint8_t led_n){
    set(led_n, HIGH);
}

void McpLeds::on_only(uint8_t led_n){
    off_all();
    on(led_n);
}

void McpLeds::off_all(){
    for(int i=0; i<_leds_n; i++) off(i);
}
