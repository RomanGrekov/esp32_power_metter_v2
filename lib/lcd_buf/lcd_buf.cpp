#include "lcd_buf.h"

LcdBuf::LcdBuf(LiquidCrystal_I2C &lcd)
: _lcd(lcd)
{

}

void LcdBuf::define_mutex(SemaphoreHandle_t mutex){
    xMutexI2c = mutex;
}

void LcdBuf::Show(){
    int row=0;
    int col=0;
    int pos=0;
    bool screen_was_updated = false;
    if (lcd_buf_changed && (xMutexI2c == NULL || xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE)){
        lcd_buf_changed = false;
        //Serial.printf("%s\n", buffer);
        _lcd.setCursor(col, row);
        while(pos < MAX_SCREEN_R * MAX_SCREEN_C && row < MAX_SCREEN_R){
            if (buffer[pos] == '\0' || buffer[pos] == '\n'){
                while(col < MAX_SCREEN_C){
                    _lcd.write(' ');
                    col++;
                }
            }
            else {
                _lcd.write(buffer[pos]);
            }
            pos++;
            col++;
            if(col >= MAX_SCREEN_C){
                row++;
                col=0;
                if (row < MAX_SCREEN_R) _lcd.setCursor(col, row);
            }
        }
        _lcd.display();
        screen_was_updated = true;
        if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
    }
    if ((cursor.is_blinking || cursor.is_underscore) && (xMutexI2c == NULL || xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE)){
        if (cursor.x != cursor.x_old || cursor.y != cursor.y_old || screen_was_updated){
            _lcd.setCursor(cursor.x, cursor.y);
        }
        if (cursor.x != cursor.x_old || cursor.y != cursor.y_old){
            cursor.x_old = cursor.x;
            cursor.y_old = cursor.y;
        }
        if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
    }
}

void LcdBuf::print(int row, const char *data, ...){
    va_list args;
    va_start(args, data);
    int res = vsprintf(get_buffer(row), data, args);
    va_end(args);

    if ((res+row*MAX_SCREEN_C) >= MAX_SCREEN_R*MAX_SCREEN_C) Serial.print("ERROR: More than buffer size written to LCD");
    lcd_buf_changed = true;
}

void LcdBuf::printarray(int row, uint8_t *data, int size){
    for(int i=0; i<size; i++){
        if ((row*MAX_SCREEN_C+i) >= (MAX_SCREEN_R*MAX_SCREEN_C)) break;
        *(get_buffer(row)+i) = data[i];
    }
    lcd_buf_changed = true;
}

void LcdBuf::printsymb(int col, int row, uint8_t symb){
    *(buffer+row*MAX_SCREEN_C+col) = symb;
    lcd_buf_changed = true;
}

char *LcdBuf::get_buffer(int row){
    return buffer+row*MAX_SCREEN_C;
}

void LcdBuf::clear(){
    for (int i=0; i<MAX_SCREEN_R; i++){
        buffer[i*MAX_SCREEN_C] = '\0';
    }
}

void LcdBuf::blink_cursor_on_off(bool on){
    cursor.is_blinking = on;
    if (xMutexI2c == NULL || xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        if (on) _lcd.blink();
        else _lcd.noBlink();
        if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
    }
}

void LcdBuf::underscore_cursor_on_off(bool on){
    cursor.is_underscore = on;
    if (xMutexI2c == NULL || xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        if (on) _lcd.cursor();
        else _lcd.noCursor();
        if (xMutexI2c != NULL) xSemaphoreGive(xMutexI2c);
    }
}

void LcdBuf::cursor_pos(int col, int row){
    cursor.x=col;
    cursor.y=row;
}
