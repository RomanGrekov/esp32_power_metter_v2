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
//#include <LiquidMenu.h>
#include "mymicromenu.h"
#include "mymenu.h"
#include <string.h>
#include "eeprom_cli.h"
#include "confd.h"

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

char lcd_buf[MAX_SCREEN_R][MAX_SCREEN_C];

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
    lcd.setCursor(0, 0);
    lcd.print("Test program");
    lcd.display();

    bool lcd_buf_changed = true;
    char old_lcd_buf[MAX_SCREEN_R][MAX_SCREEN_C];

    while(1){
        lcd_buf_changed = false;
        for(int i=0; i<MAX_SCREEN_R; i++){
            for(int i1=0; i1<MAX_SCREEN_C; i1++){
                if (lcd_buf[i][i1] != old_lcd_buf[i][i1]){
                    old_lcd_buf[i][i1] = lcd_buf[i][i1];
                    lcd_buf_changed = true;
                }
            }
        }
        if (lcd_buf_changed){
            xSemaphoreTake(xMutexI2c, portMAX_DELAY);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(lcd_buf[0]);
            lcd.setCursor(0, 1);
            lcd.print(lcd_buf[1]);
            lcd.display();
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
    int cnt=0;
	if (Text){
        strcpy(lcd_buf[0], Text);
        lcd_buf[1][0] = '\0';
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
    char num[16];

    Menu_Item_t *cur_menu = Menu_GetCurrentMenu();
    strcpy(lcd_buf[1], "Address: ");
    itoa(confd.get_address(cur_menu->index), num, 10);
    strcpy(lcd_buf[1]+8, num);
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

    strcpy(lcd_buf[0], "Searching sensors");
    lcd_buf[1][0] = '\0';

    // Init pzem
    #define MAX_DEVS 0xf8
    #define CMD_RIR         0X04
    PZEM004Tv30 pzem(&Serial2);
    // Search device
    uint8_t response[7];
    strcpy(lcd_buf[1], "Try addr: ");
    for(uint8_t addr = 0x01; addr <= MAX_DEVS; addr++){
        itoa(addr, num, 10);
        strcpy(lcd_buf[1]+10, num);
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
        Log.error("Failed to find any device"CR);
        strcpy(lcd_buf[0], "Faile to find");
        strcpy(lcd_buf[1], "any device");
        vTaskDelay(2000);
    }
    else {
        Log.notice("Found device %d" CR, dev_addr);
        strcpy(lcd_buf[0], "Saving...");
        strcpy(lcd_buf[1]+11, " Ok");
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
                strcpy(lcd_buf[0], "Failed to save");
                strcpy(lcd_buf[1], "sensor to store");
            }
            else{
                strcpy(lcd_buf[0], "Success!");
            }
        }
        else{
            strcpy(lcd_buf[0], "Failed to set");
            strcpy(lcd_buf[1], "new address");
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

    strcpy(lcd_buf[0], "Are you shure?");
    strcpy(lcd_buf[1], "->No Yes");
    while(1){
        if (enc.isChanged()){
            if (enc.getDirection() == cw){
                strcpy(lcd_buf[1], "No ->Yes");
                shure=true;
            }
            else{
                strcpy(lcd_buf[1], "->No Yes");
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
                    strcpy(lcd_buf[0], "Removed sensor");
                    itoa(current_sensor_n, num, 10);
                    strcpy(lcd_buf[1], num);
                    lcd_buf[1][2] = '\0';
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
            strcpy(lcd_buf[0], "Sensor absent");
            strcpy(lcd_buf[1], "please add it!");
        }
        else{
            PZEM004Tv30 pzem(&Serial2, addr);
            voltage = pzem.voltage();
            current = pzem.current();
            energy = pzem.energy();
            sprintf(lcd_buf[0], "V %.1f A %.2f", voltage, current);
            sprintf(lcd_buf[1], "E %.2f", energy);
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
    //child_menu->index = _index;
    /*
    uint8_t cur_sensor=0;
    bool cur_sensor_changed=true;
    bool old_btn_state=false;
    bool btn_state=false;

    while(1){
        if (enc.isChanged()){
            if (enc.getDirection() == cw){
                if(cur_sensor < SENSORS_AMOUNT-1) cur_sensor++;
                else cur_sensor = 0;
                cur_sensor_changed = true;
            }
            else{
                if(cur_sensor > 0) cur_sensor--;
                else cur_sensor = SENSORS_AMOUNT-1;
                cur_sensor_changed = true;
            }
        }
        btn_state = enc_btn.isPressed();
        if (btn_state != old_btn_state){
            old_btn_state = btn_state;
            if (btn_state == 0){
        		//Menu_Navigate(MENU_PARENT);
        		//Menu_EnterItem(KEY_NEXT);
                break;
            }
        }
        if (cur_sensor_changed){
            cur_sensor_changed = false;
        }
        vTaskDelay(100);
    }
	Menu_Navigate(MENU_NEXT);
    */

}
