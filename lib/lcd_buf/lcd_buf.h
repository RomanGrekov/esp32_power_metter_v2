#ifndef LCD_BUF
#define LCD_BUF

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <stdarg.h>

#define MAX_SCREEN_R 2
#define MAX_SCREEN_C 16

struct Cursor {
    int x=0;
    int y=0;
    int x_old=0;
    int y_old=0;
    bool is_blinking=false;
    bool is_underscore=false;
};

class LcdBuf
{
public:
    LcdBuf(LiquidCrystal_I2C &lcd);
    bool is_changed();
    void Show();
    void print(int row, const char *data, ...);
    void printarray(int row, uint8_t *data, int size);
    void printsymb(int row, int col, uint8_t symb);
    void clear();

    void blink_cursor_on_off(bool on);
    void underscore_cursor_on_off(bool on);
    void cursor_pos(int col, int row);

private:
    LiquidCrystal_I2C& _lcd;
    bool lcd_buf_changed = false;
    char old_buffer[MAX_SCREEN_R*MAX_SCREEN_C];
    char buffer[MAX_SCREEN_R*MAX_SCREEN_C];
    Cursor cursor;

    char *get_buffer(int row=0);
    void copy(char *src, char *dst, int size);
};

#endif
