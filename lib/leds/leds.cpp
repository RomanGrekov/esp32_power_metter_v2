#include "leds.h"

McpLeds::McpLeds(int address, uint8_t leds_n)
{
  _address = address;
  _leds_n = leds_n;
}

void McpLeds::define_mutex(SemaphoreHandle_t mutex){
    xMutexI2c = mutex;
}

void McpLeds::init(){
    if (xMutexI2c == NULL || xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        this->mcp.begin(_address);
        for(int i=0; i<_leds_n; i++){
            this->mcp.pinMode(i, OUTPUT);
        }
        if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
    }
}

void McpLeds::set(uint8_t led_n, uint8_t state){
    if (xMutexI2c == NULL || xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        this->mcp.digitalWrite(led_n, state);
        if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
    }
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
    //for(int i=0; i<_leds_n; i++) off(i);
    if (xMutexI2c == NULL || xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        this->mcp.writeGPIOAB(0x0);
        if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
    }
}
