#include "lcd_buf.h"

LcdBuf::LcdBuf(LiquidCrystal_I2C &lcd)
: _lcd(lcd)
{

}

bool LcdBuf::is_changed(){
    bool tmp;
    for(int i=0; i<MAX_SCREEN_R * MAX_SCREEN_C; i++){
        if (buffer[i] != old_buffer[i]){
            old_buffer[i] = buffer[i];
            lcd_buf_changed = true;
        }
    }
    tmp = lcd_buf_changed;
    lcd_buf_changed = false;
    return tmp;
}

void LcdBuf::show(){
    int row=0;
    int col=-1;
    int pos=0;
    //_lcd.clear();
    while(pos < MAX_SCREEN_R * MAX_SCREEN_C){
        if(col >= (MAX_SCREEN_C-1)){
            row++;
            col=0;
        }
        else col++;
        //pos = col + row*MAX_SCREEN_C;
        if ((col + row*MAX_SCREEN_C) >= MAX_SCREEN_R * MAX_SCREEN_C) break;
        if (buffer[pos] == '\0' || buffer[pos] == '\n'){
            while(col < MAX_SCREEN_C){
                _lcd.write(' ');
                col++;
            }
            col = MAX_SCREEN_C;
        }
        else {
            _lcd.setCursor(col, row);
            _lcd.write(buffer[pos]);
        }
        if (buffer[pos] == '\0') pos = col + row*MAX_SCREEN_C;
        else pos++;
    }
    _lcd.display();
}

void LcdBuf::print(int row, const char *data, ...){
    va_list args;
    va_start(args, data);
    vsprintf(get_buffer(row), data, args);
    va_end(args);
}

char *LcdBuf::get_buffer(int row){
    return buffer+row*MAX_SCREEN_C;
}

void LcdBuf::clear(){
    for (int i=0; i<MAX_SCREEN_R; i++){
        buffer[i*MAX_SCREEN_C] = '\0';
    }
}
