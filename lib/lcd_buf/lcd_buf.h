#ifndef LCD_BUF
#define LCD_BUF

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <stdarg.h>

#define MAX_SCREEN_R 2
#define MAX_SCREEN_C 16

class LcdBuf
{
public:
    LcdBuf(LiquidCrystal_I2C &lcd);
    bool is_changed();
    void show();
    void print(int row, const char *data, ...);
    void clear();

private:
    LiquidCrystal_I2C& _lcd;
    bool lcd_buf_changed = false;
    char old_buffer[MAX_SCREEN_R*MAX_SCREEN_C];
    char buffer[MAX_SCREEN_R*MAX_SCREEN_C];

    char *get_buffer(int row=0);
    void copy(char *src, char *dst, int size);
};

#endif
