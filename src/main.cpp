#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include "PZEM004Tv30.h"
#include <Pushbutton.h>
#include <ArduinoLog.h>
#include "FreeRTOS.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "html.h"
#include "encoder.h"
#include <LiquidCrystal_I2C.h>
#include "mymicromenu.h"
#include "mymenu.h"
#include <string.h>
#include "eeprom_cli.h"
#include "confd.h"
#include "lcd_buf.h"

/*
    Encoder pins
*/
#define ENC_A_PIN 39
#define ENC_B_PIN 36
#define ENC_BTN_PIN 34
Encoder enc(ENC_A_PIN, ENC_B_PIN);
Pushbutton enc_btn(ENC_BTN_PIN);

/*
    LCD screen and text menu definition
*/
#define MAX_SCREEN_R 2
#define MAX_SCREEN_C 16
#define SCREEN_ADDR 0x27
LiquidCrystal_I2C lcd(SCREEN_ADDR, MAX_SCREEN_C, MAX_SCREEN_R);
LcdBuf lcd_buffer(lcd);


/*
    RTOS tasks prototypes
*/
void taskControlLed( void * parameter );
void taskLcd( void * parameter );
void taskEncoderRead( void * parameter);

/*
    Eeprom init
*/
//EepromCli eeprom_cli(21, 22, 0x50);
EepromCli eeprom_cli(0x50);
Confd confd(eeprom_cli);

/*
    Mutex needed to make access to I2C bus atomic
*/
SemaphoreHandle_t xMutexI2c;

/*
    Current selected sensor number in lcd menu
*/
int current_sensor_n=0;

void setup() {
    /*
        Setup logger
    */
    Serial.begin(9600);
    while(!Serial && !Serial.available()){}
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.notice("###### Start logger ######"CR);

    /*
        Setup encoder
    */
    enc.begin([]{enc.EncChanged_ISR();});
    //enc.setRotation(-10, 10, false);

	/* Set up the default menu text write callback, and navigate to an absolute menu item entry. */
	Menu_SetGenericWriteCallback(Generic_Write);
	Menu_Navigate(&Menu_1);

    /*
        Mutex needed to make access to I2C bus atomic
    */
    xMutexI2c = xSemaphoreCreateMutex();

    /*
        Declare RTOS tasks
    */
    xTaskCreate(taskControlLed,
                "TaskLedBlinker",
                1000,
                NULL,
                1,
                NULL);
    xTaskCreate(taskEncoderRead,
                "EncReadTask",
                10000,
                NULL,
                1,
                NULL);
    xTaskCreate(taskLcd,
                "Lcd_show",
                10000,
                NULL,
                1,
                NULL);
}

void loop() {
}

void taskControlLed( void * parameter ){
    /*
        Control led settings
    */
    #define LED_PIN 23
    #define LED_DEFAULT_F 1 // Hz
    #define LED_FILE_UPLOADING_F 100
    int Led_f = LED_DEFAULT_F;
    bool ledstatus = false;

    pinMode(LED_BUILTIN, OUTPUT);
    while(1){
        ledstatus = !ledstatus;
        digitalWrite(LED_BUILTIN, ledstatus);
        vTaskDelay(1000/Led_f);
    }
}

void taskEncoderRead( void * parameter ) {
    bool old_btn_state=false;
    bool btn_state=false;

    while(1){
        if (enc.isChanged()){
            if (enc.getDirection() == cw){
                Log.notice("Enc: ->"CR);
    			Menu_Navigate(MENU_NEXT);
            }
            else{
                Log.notice("Enc: <-"CR);
    			Menu_Navigate(MENU_PREVIOUS);
            }

            //Log.notice("%d"CR, enc.getRotation());
        }
        btn_state = enc_btn.isPressed();
        if (btn_state != old_btn_state){
            Log.notice("Btn pressed: %d" CR, btn_state);
            old_btn_state = btn_state;
            if (btn_state == 0){
        		Menu_EnterItem(KEY_NEXT);
        		Menu_Navigate(MENU_CHILD);
            }
        }
        vTaskDelay(100);
    }
}

void taskLcd( void * parameter ) {
    /*
        Setup LCD
        MUST to be called here. Otherwise LCD gluchit
    */
    lcd.init();
    lcd.backlight();

    while(1){
        if (lcd_buffer.is_changed()){
            xSemaphoreTake(xMutexI2c, portMAX_DELAY);
            lcd_buffer.show();
            xSemaphoreGive(xMutexI2c);
        }
        vTaskDelay(100);
    }
}

/** Example menu item specific enter callback function, run when the associated menu item is entered. */
static void Level1Item1_Select(int parent_index)
{
	Log.notice("Select"CR);
}

/** Example menu item specific select callback function, run when the associated menu item is selected. */
static void Level1Item1_Enter(Key_Pressed_t key)
{
	Log.notice("Enter"CR);
}

/** Generic function to write the text of a menu.
 *
 *  \param[in] Text   Text of the selected menu to write, in \ref MENU_ITEM_STORAGE memory space
 */
static void Generic_Write(const char* Text)
{
	if (Text){
        lcd_buffer.clear();
        lcd_buffer.print(0, Text);
    }
}

void lcd_print(const char* _string){

}

static void Read_Sensors_Enter(Key_Pressed_t key){
    /*
        Add test sensor
    */
    //uint8_t res = confd.store_sensors();
    //if (res != 0) Log.error("Failed to write sensor addresses" CR);

    /*
        Initialise sensors
    */
    Log.notice("Reading sensors addresses");
    for (int i=0; i < SENSORS_AMOUNT; i++) confd.add_sensor(i, 0);
    xSemaphoreTake(xMutexI2c, portMAX_DELAY);
    uint8_t res = confd.read_sensors();
    if (res != 0) Log.error("Failed to read sensor addresses" CR);
    xSemaphoreGive(xMutexI2c);
    Log.notice("Done");

}
static void sensor_select(int parent_index){
    /*
        Menu of sensors
        Show sensor id and it's address in eeprom
    */
    Menu_Item_t *cur_menu = Menu_GetCurrentMenu();
    lcd_buffer.print(1, "Address: %d", confd.get_address(cur_menu->index));
}

static void sensor_add_select(int parent_index){
    /*
        Needed to pass selected sensor number deeper to other menus like del, show...
    */
    current_sensor_n = parent_index;
}


static void sensor_add(Key_Pressed_t key){
    bool res;
    uint8_t res_int;
    char num[16];
    uint8_t dev_addr=0;

    lcd_buffer.print(0, "Searching sensors");

    // Init pzem
    #define MAX_DEVS 0xf8
    #define CMD_RIR         0X04
    PZEM004Tv30 pzem(&Serial2);
    // Search device
    uint8_t response[7];
    for(uint8_t addr = 0x01; addr <= MAX_DEVS; addr++){
        lcd_buffer.print(1, "Try addr: %d", addr);
        pzem.sendCmd8(CMD_RIR, 0x00, 0x01, false, addr);
        if(pzem.recieve(response, 7) != 7){ // Something went wrong
            vTaskDelay(100);
            continue;
        } else {
            dev_addr = addr;
            break;
        }
        vTaskDelay(100);
    }

    if (dev_addr == 0){
        Log.error("Failed to find any device" CR);
        lcd_buffer.print(0, "Failed to find any device");
        vTaskDelay(2000);
    }
    else {
        Log.notice("Found device %d" CR, dev_addr);
        lcd_buffer.print(0, "Saving...\nOk");
        vTaskDelay(2000);
        PZEM004Tv30 pzem(&Serial2, dev_addr);
        res = pzem.setAddress(current_sensor_n+1);
        if (res){
            Log.notice("Storing sensor..." CR);
            confd.add_sensor(current_sensor_n, current_sensor_n+1);
            xSemaphoreTake(xMutexI2c, portMAX_DELAY);
            res_int = confd.store_sensors();
            xSemaphoreGive(xMutexI2c);
            if (res_int != 0){
                Log.error("Failed to store sensors");
                lcd_buffer.print(0, "Failed to save sensor to mem");
            }
            else{
                lcd_buffer.clear();
                lcd_buffer.print(0, "Success!");
            }
        }
        else{
            //lcd_buffer.print(0, "Failed to set new address");
        }
    }
    bool old_btn_state=false;
    bool btn_state=false;
    while(1){
        btn_state = enc_btn.isPressed();
        if (btn_state != old_btn_state){
            old_btn_state = btn_state;
            if (btn_state == 0){
                break;
            }
        }
        vTaskDelay(100);
    }
    Menu_Navigate(&Menu_2_1);
}

static void sensor_del(Key_Pressed_t key){
    bool old_btn_state=false;
    bool btn_state=false;
    bool shure=false;
    char num[16];

    lcd_buffer.print(0, "Are you shure?\n->No Yes");
    while(1){
        if (enc.isChanged()){
            if (enc.getDirection() == cw){
                lcd_buffer.print(1, "No ->Yes");
                shure=true;
            }
            else{
                lcd_buffer.print(1, "->No Yes");
                shure=false;
            }
        }
        btn_state = enc_btn.isPressed();
        if (btn_state != old_btn_state){
            old_btn_state = btn_state;
            if (btn_state == 0){
                if (shure){
                    confd.add_sensor(current_sensor_n, 0);
                    confd.store_sensors();
                    lcd_buffer.print(0, "Removed sensor\n- %d", current_sensor_n);
                    vTaskDelay(2000);
                }
                break;
            }
        }
        vTaskDelay(100);
    }
    Menu_Navigate(&Menu_2_1);
}
static void sensor_show(Key_Pressed_t key){
    bool old_btn_state=false;
    bool btn_state=false;
    float voltage;
    float current;
    float energy;
    uint8_t addr = confd.get_address(current_sensor_n);
    while(1){
        if (addr == 0){
            lcd_buffer.print(0, "Sensor absent\nplease add it!");
        }
        else{
            PZEM004Tv30 pzem(&Serial2, addr);
            voltage = pzem.voltage();
            current = pzem.current();
            energy = pzem.energy();
            lcd_buffer.print(0, "V %.1f A %.2f\nE %.2f", voltage, current, energy);
        }
        btn_state = enc_btn.isPressed();
        if (btn_state != old_btn_state){
            old_btn_state = btn_state;
            if (btn_state == 0){
                break;
            }
        }
        vTaskDelay(100);
    }
    Menu_Navigate(&Menu_2_1);
}

static void sensor_enter(Key_Pressed_t key){
}
