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
#include "leds.h"
#include "sensors.h"
#include <ESPDateTime.h>

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
    Eeprom init
*/
//EepromCli eeprom_cli(21, 22, 0x50);
EepromCli eeprom_cli(0x50);
Confd confd(eeprom_cli);

/*
   Encoder actions
*/
enum EncActions{
    encActionBtnPressed=1,
    encActionCwMove,
    encActionCcwMove,
    encActionBtnLongPressed
};

/*
    Current selected sensor number in lcd menu
*/
int current_sensor_n=0;

/*
    Sensors addreses
*/
#define MAX_SENSORS_AMOUNT 10
uint8_t sensors[MAX_SENSORS_AMOUNT];
/*
  Array of all sensors data
*/
Sensor sensors_data[MAX_SENSORS_AMOUNT];
/*
  Mcp23017 connected leds
*/
McpLeds Leds(0, MAX_SENSORS_AMOUNT);

/*
    Alphabet for Wifi
*/
const char standard_alphabet[] = " ._-0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
#define standard_alphabet_size sizeof(standard_alphabet)-1 // Last symbol is always \0

/*
  Mutex to block reading when write
*/
SemaphoreHandle_t xMutexSensorRead;

/*
    Create queue object for encoder
*/
QueueHandle_t encActionsQueue;

/*
    Mutex needed to make access to I2C bus atomic
*/
SemaphoreHandle_t xMutexI2c;

/*
    Semaphore to say that sensors changed
*/
xSemaphoreHandle SensorsChangedSemaphore;

/*
    RTOS tasks prototypes
*/
void taskControlLed( void * parameter );
void taskLcd( void * parameter );
void taskEncoderRead( void * parameter);
void taskEncoderBtnRead( void * parameter);
void taskMenu( void * parameter );
void taskSensorsRead( void * parameter );
void taskDetectCharging( void * parameter );
void taskChargingLeds( void * parameter );
void taskChargingStats( void * parameter );
void edit_menu(uint8_t *data, int len, const char *alphabet, int alphabet_len, bool do_reset_data);

void setup() {
    /*
        Setup logger
    */
    Serial.begin(9600);
    while(!Serial && !Serial.available()){}
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.notice("###### Start logger ######" CR);

    /*
        Setup encoder
    */
    enc.begin([]{enc.EncChanged_ISR();});
    //enc.setRotation(-10, 10, false);

  	/*
      Set up the default menu text write callback, and navigate to an absolute menu item entry.
    */
  	Menu_SetGenericWriteCallback(Generic_Write);
  	Menu_Navigate(&Menu_1);

    /*
        Mutex needed to make access to I2C bus atomic
    */
    xMutexI2c = xSemaphoreCreateMutex();

    /*
      Mutex to block sensor reading when write
    */
    xMutexSensorRead = xSemaphoreCreateMutex();

    /*
        Initialize encoder button queue
    */
    encActionsQueue = xQueueCreate(10, sizeof(EncActions));

    /*
        Initialize binary semaphore for sensors changed
    */
    vSemaphoreCreateBinary(SensorsChangedSemaphore);
    xSemaphoreGive(SensorsChangedSemaphore);


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
                1000,
                NULL,
                1,
                NULL);
    xTaskCreate(taskEncoderBtnRead,
                "EncBtnReadTask",
                1000,
                NULL,
                1,
                NULL);
    xTaskCreate(taskLcd,
                "Lcd_show",
                10000,
                NULL,
                1,
                NULL);
    xTaskCreate(taskMenu,
                "Menu handle",
                10000,
                NULL,
                1,
                NULL);
    xTaskCreate(taskSensorsRead,
                "Read sensors task",
                10000,
                NULL,
                1,
                NULL);
    xTaskCreate(taskDetectCharging,
                "Detect if charging in progress",
                1000,
                NULL,
                1,
                NULL);
    xTaskCreate(taskChargingLeds,
                "Enable leds when charging",
                10000,
                NULL,
                1,
                NULL);
    xTaskCreate(taskChargingStats,
                "Save chargigs to struct",
                1000,
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
    EncActions enc_actions;
    while(1){
        if (enc.isChanged()){
            if (enc.getDirection() == cw){
                Log.notice("Enc: ->" CR);
                enc_actions = encActionCwMove;
            }
            else{
                Log.notice("Enc: <-" CR);
                enc_actions = encActionCcwMove;
            }
            xQueueSend(encActionsQueue, &enc_actions, portMAX_DELAY);
        }
        vTaskDelay(10);
    }
}

void taskEncoderBtnRead( void * parameter ) {
    EncActions enc_actions;
    bool btn_was_pressed=false;
    bool btn_was_long_pressed=false;
    uint16_t start_millis;
    uint16_t _millis;

    while(1){
        if (enc_btn.getSingleDebouncedPress() && ! btn_was_long_pressed){
            if (!btn_was_pressed){
                btn_was_pressed = true;
                start_millis = millis();
            }
        }
        if (btn_was_pressed && ! btn_was_long_pressed){
            _millis = millis() - start_millis;
            if (enc_btn.getSingleDebouncedRelease() && btn_was_pressed){
                btn_was_pressed = false;
                if (20 < _millis && _millis < 800){
                    Log.notice("Btn pressed: Short" CR);
                    enc_actions = encActionBtnPressed;
                    xQueueSend(encActionsQueue, &enc_actions, portMAX_DELAY);
                }
            }
            if (800 <= _millis && _millis < 30000 && btn_was_pressed){
                Log.notice("Btn pressed: Long" CR);
                enc_actions = encActionBtnLongPressed;
                xQueueSend(encActionsQueue, &enc_actions, portMAX_DELAY);
                btn_was_long_pressed = true;
            }
        }
        if (btn_was_long_pressed){
            if (enc_btn.getSingleDebouncedRelease() && btn_was_pressed){
                btn_was_long_pressed = false;
                btn_was_pressed = false;
            }
        }
        vTaskDelay(10);
    }
}

void taskMenu( void * parameter ) {
    EncActions enc_action;
    while(1){
        if (uxQueueMessagesWaiting(encActionsQueue) > 0){
            xQueueReceive(encActionsQueue, &enc_action, portMAX_DELAY);
            switch(enc_action){
                case encActionCwMove:
			         Menu_Navigate(MENU_NEXT);
                break;
                case encActionCcwMove:
			         Menu_Navigate(MENU_PREVIOUS);
                break;
                case encActionBtnPressed:
            		Menu_EnterItem(KEY_NEXT);
            		Menu_Navigate(MENU_CHILD);
                break;
            }
        }
        vTaskDelay(100);
    }
}

//void vCallbackLcdBlink( xTimerHandle xTimer ){
//    lcd_buffer.DoBlink();
//}

void taskLcd( void * parameter ) {
    bool is_blinking_old=false;
    xTimerHandle blink_timer;
    const uint32_t timer_id=30;
    /*
        Setup LCD
        MUST to be called here. Otherwise LCD gluchit
    */
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        lcd.init();
        lcd.backlight();
        xSemaphoreGive(xMutexI2c);
    }

    while(1){
        if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
            lcd_buffer.Show();
            xSemaphoreGive(xMutexI2c);
        }
        //if (is_blinking_old != lcd_buffer.get_cursor_blink()){
        //    is_blinking_old = lcd_buffer.get_cursor_blink();
        //    if (is_blinking_old){
        //        Log.notice("Turn on timer" CR);
        //        blink_timer = xTimerCreate("Lcd symb blink", pdMS_TO_TICKS(500),
        //                                   pdTRUE, (void *)timer_id, &vCallbackLcdBlink);
        //        if (blink_timer == NULL) Log.error("Failed to create timer for lcd blink: %d" CR, timer_id);
        //        xTimerStart(blink_timer, 0);
//
        //    } else {
        //        Log.notice("Turn off timer" CR);
        //        BaseType_t res = xTimerStop(blink_timer, pdMS_TO_TICKS(100));
        //        if (res != pdPASS) Log.error("Failed to stop timer for lcd blink: %d" CR, timer_id);
        //    }
        //}

        vTaskDelay(100);
    }
}

void taskSensorsRead( void * parameter ) {
    uint8_t addr=0;
    int index_n=0;
    uint8_t indexes[MAX_SENSORS_AMOUNT];

    PZEM004Tv30 pzem(&Serial2, addr); // Fake init
    uint8_t ws_amount=0;
    while(1){
      // If something changed (added/deleted sensor) than read sensors again
      // otherwise just wait timeout
      if (xSemaphoreTake(SensorsChangedSemaphore, 10) == pdPASS){
          /*
              Read sensors addresses from eeprom
          */
          Read_Sensors_Enter(KEY_OK);
          /*
              Find working sensors
          */
          ws_amount = get_sensors_indexes(sensors, indexes, MAX_SENSORS_AMOUNT);

          if (ws_amount == 0 ){
              Log.error( "No one working sensors to show" CR);
              vTaskDelay(1000);
          }
          index_n=0;

      }
      if (ws_amount <= 0){
          vTaskDelay(pdMS_TO_TICKS(500));
          continue;
      }

      addr = sensors[indexes[index_n]];
      xSemaphoreTake(xMutexSensorRead, portMAX_DELAY);
      pzem.init(addr);
      sensors_data[indexes[index_n]].address = addr;
      sensors_data[indexes[index_n]].V = pzem.voltage();
      sensors_data[indexes[index_n]].A = pzem.current();
      sensors_data[indexes[index_n]].Kwh = pzem.energy();
      xSemaphoreGive(xMutexSensorRead);

      if (index_n < ws_amount-1) index_n++;
      else {
          index_n = 0;
          vTaskDelay(pdMS_TO_TICKS(500));
      }
    }
}

void vCallbackNotCharging( xTimerHandle xTimer ){
    uint32_t timer_id;
    timer_id = (uint32_t)pvTimerGetTimerID(xTimer);
    sensors_data[timer_id].is_charging = false;
}

void taskDetectCharging( void * parameter ) {
    #define CHARGING_TRASHOLD 60 // Seconds
    xTimerHandle timers[MAX_SENSORS_AMOUNT];

    while (1) {
        for (uint32_t id=0; id<MAX_SENSORS_AMOUNT; id++){
            if (sensors_data[id].A > 0){
                sensors_data[id].is_charging = true;
                if(xTimerIsTimerActive(timers[id]) != pdFALSE){
                    BaseType_t res = xTimerStop(timers[id], pdMS_TO_TICKS(100));
                    if (res != pdPASS) Log.error("Failed to stop timer for sensor: %d" CR, id);
                }
            }else{
                if(sensors_data[id].is_charging){
                    timers[id] = xTimerCreate("Is chargin treshold", pdMS_TO_TICKS(CHARGING_TRASHOLD * 1000),
                                             pdFALSE, (void *)id, &vCallbackNotCharging);
                    if (timers[id] == NULL) Log.error("Failed to create timer for sensor: %d" CR, id);
                }
            }
        }
        vTaskDelay(100);
    }
}

void taskChargingLeds( void * parameter ) {
    /*
        Start leds
    */
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        Leds.init();
        xSemaphoreGive(xMutexI2c);
        vTaskDelay(100);
    }

    while(1){
        if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
            for (int id=0; id<MAX_SENSORS_AMOUNT; id++){
                if (sensors_data[id].is_charging) Leds.on(id);
                else Leds.off(id);
                vTaskDelay(10);
            }
            xSemaphoreGive(xMutexI2c);
        }
        vTaskDelay(100);
    }
}

void taskChargingStats( void * parameter ) {
    OneCharge _chargings[MAX_SENSORS_AMOUNT];
    while(1){
        for (int id=0; id<MAX_SENSORS_AMOUNT; id++){
            if(sensors_data[id].is_charging){
                // Search for started chargings
                if(_chargings[id].start > 0) continue;
                else {
                    _chargings[id].start = DateTime.now();
                    _chargings[id].start_kwh = sensors_data[id].Kwh;
                }
            } else {
                if(_chargings[id].start > 0){
                    _chargings[id].finish = DateTime.now();
                    _chargings[id].finish_kwh = sensors_data[id].Kwh;
                }
            }
            if (_chargings[id].start > 0 && _chargings[id].finish > 0){
                sensors_data[id].push_one_charging(_chargings[id]);
                _chargings[id].start =      0;
                _chargings[id].finish =     0;
                _chargings[id].start_kwh =  0;
                _chargings[id].finish_kwh = 0;
            }
        }
        vTaskDelay(100);
    }
}

/** Example menu item specific enter callback function, run when the associated menu item is entered. */
static void Level1Item1_Select(int parent_index)
{
	Log.notice("Select" CR);
}

/** Example menu item specific select callback function, run when the associated menu item is selected. */
static void Level1Item1_Enter(Key_Pressed_t key)
{
	Log.notice("Enter" CR);
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

static void Read_Sensors_Enter(Key_Pressed_t key){
    /*
        Add test sensor
    */
    //uint8_t res = confd.store_sensors();
    //if (res != 0) Log.error("Failed to write sensor addresses" CR);

    /*
        Initialise sensors
    */
    Log.notice("Reading sensors addresses" CR);
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        uint8_t res = confd.read_sensors(sensors);
        if (res != 0) Log.error("Failed to read sensor addresses" CR);
        xSemaphoreGive(xMutexI2c);
    }
    else vTaskDelay(10);
    Log.notice("Done" CR);
}

static void sensor_select(int parent_index){
    /*
        Menu of sensors
        Show sensor id and it's address in eeprom
    */
    Menu_Item_t *cur_menu = Menu_GetCurrentMenu();
    lcd_buffer.print(1, "Address: %d", sensors[cur_menu->index]);
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
            sensors[current_sensor_n] = current_sensor_n+1;
            bool done=false;
            while (!done){
                if (xSemaphoreTake(xMutexI2c, pdMS_TO_TICKS(100)) == pdTRUE){
                    res_int = confd.store_sensors(sensors);
                    xSemaphoreGive(xMutexI2c);
                    done = true;
                }
                else vTaskDelay(10);
            }
            if (res_int != 0){
                Log.error("Failed to store sensors");
                lcd_buffer.print(0, "Failed to save sensor to mem");
            }
            else{
                lcd_buffer.clear();
                lcd_buffer.print(0, "Success!");
                if (xSemaphoreGive(SensorsChangedSemaphore) != pdTRUE) Log.error("Failed to give sensors semaphore" CR);
            }
        }
        else{
            lcd_buffer.print(0, "Failed to set new address");
        }
    }
    EncActions enc_action;
    while(1){
        if (uxQueueMessagesWaiting(encActionsQueue) > 0){
            xQueueReceive(encActionsQueue, &enc_action, portMAX_DELAY);
            if (enc_action == encActionBtnPressed) break;
        }
        vTaskDelay(100);
    }
    Menu_Navigate(&Menu_2_1);
}

static void sensor_del(Key_Pressed_t key){
    bool shure=false;
    EncActions enc_action;

    lcd_buffer.print(0, "Are you shure?\n->No Yes");
    while(1){
        if (uxQueueMessagesWaiting(encActionsQueue) > 0){
            xQueueReceive(encActionsQueue, &enc_action, portMAX_DELAY);
            switch(enc_action){
                case encActionCwMove:
                    lcd_buffer.print(1, "No ->Yes");
                    shure=true;
                break;
                case encActionCcwMove:
                    lcd_buffer.print(1, "->No Yes");
                    shure=false;
                break;
                case encActionBtnPressed:
                    if (shure){
                        sensors[current_sensor_n] = 0;
                        confd.store_sensors(sensors);
                        lcd_buffer.print(0, "Removed sensor\n- %d", current_sensor_n);
                        if (xSemaphoreGive(SensorsChangedSemaphore) != pdTRUE) Log.error("Failed to give sensors semaphore" CR);
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
    EncActions enc_action;
    uint8_t addr = sensors[current_sensor_n];
    while(1){
        if (addr == 0){
            lcd_buffer.print(0, "Sensor absent\nplease add it!");
        }
        else{
            xSemaphoreTake(xMutexSensorRead, portMAX_DELAY);
            lcd_buffer.print(0, "V %.1f A %.2f\nE %.2f", sensors_data[current_sensor_n].V,
                                                         sensors_data[current_sensor_n].A,
                                                         sensors_data[current_sensor_n].Kwh);
            xSemaphoreGive(xMutexSensorRead);
        }
        if (uxQueueMessagesWaiting(encActionsQueue) > 0){
            xQueueReceive(encActionsQueue, &enc_action, portMAX_DELAY);
            if (enc_action == encActionBtnPressed) break;
        }
        vTaskDelay(100);
    }
    Menu_Navigate(&Show_sensor);
}

static void showAllSensorsRuntime(Key_Pressed_t key){
    EncActions enc_action;
    uint8_t ws_amount=0;
    uint8_t index_n=0;
    uint8_t indexes[MAX_SENSORS_AMOUNT];

    index_n=0;
    while(1){
        if (ws_amount == 0 ){
            ws_amount = get_sensors_indexes(sensors, indexes, MAX_SENSORS_AMOUNT);
            lcd_buffer.print(0, "No one working sensors to show");
            Log.error("No sensors to read" CR);
            vTaskDelay(100);
            continue;
        }
        xSemaphoreTake(xMutexSensorRead, portMAX_DELAY);
        lcd_buffer.print(0, "%d: Kw/h: %.2f\nV: %.1f A: %.2f", indexes[index_n]+1,
                                                               sensors_data[indexes[index_n]].Kwh,
                                                               sensors_data[indexes[index_n]].V,
                                                               sensors_data[indexes[index_n]].A);
        xSemaphoreGive(xMutexSensorRead);
        if (uxQueueMessagesWaiting(encActionsQueue) > 0){
            xQueueReceive(encActionsQueue, &enc_action, portMAX_DELAY);
            switch(enc_action){
                case encActionCwMove:
                    if (index_n < ws_amount-1) index_n++;
                    else index_n = 0;
                break;
                case encActionCcwMove:
                    if (index_n > 0) index_n--;
                    else index_n = ws_amount-1;
                break;
                case encActionBtnPressed:
                    break;
                break;
            }
        }
        vTaskDelay(100);
    }

    Menu_Navigate(&Menu_2);
}

static void sensor_enter(Key_Pressed_t key){
}

static void Wifi_Name_Menu_Select(int parent_index){
    uint8_t name[WIFI_NAME_ADDR_SIZE];
    uint8_t res;
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        res = confd.read_wifi_name(name);
        xSemaphoreGive(xMutexI2c);
    }
    if(res != 0){
        Log.error("Failed to read wifi name" CR);
        lcd_buffer.print(1, "N/A");
    }
    else {
        lcd_buffer.printarray(1, name, WIFI_NAME_ADDR_SIZE);
    }
}

static void Wifi_Name_Menu_Enter(Key_Pressed_t key){
    uint8_t name[WIFI_NAME_ADDR_SIZE];
    uint8_t res=1;
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        res = confd.read_wifi_name(name);
        xSemaphoreGive(xMutexI2c);
    }
    if(res != 0){
        Log.error("Failed to read wifi name" CR);
        edit_menu(name, WIFI_NAME_ADDR_SIZE, standard_alphabet, standard_alphabet_size, true);
    }
    else {
        edit_menu(name, WIFI_NAME_ADDR_SIZE, standard_alphabet, standard_alphabet_size, false);
    }
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        res = confd.store_wifi_name(name);
        xSemaphoreGive(xMutexI2c);
    }
    if (res != 0){
        lcd_buffer.print(0, "Failed!!!");
        Log.error("Failed to save wifi name" CR);
        vTaskDelay(2000);
    }
    Menu_Navigate(&Menu_1_2);
}

static void Wifi_Pw_Menu_Select(int parent_index){
    uint8_t pw[WIFI_PW_ADDR_SIZE];
    uint8_t res;
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        res = confd.read_wifi_pw(pw);
        xSemaphoreGive(xMutexI2c);
    }
    if(res != 0){
        Log.error("Failed to read wifi pw" CR);
        lcd_buffer.print(1, "N/A");
    }
    else {
        lcd_buffer.printarray(1, pw, WIFI_PW_ADDR_SIZE);
    }
}

static void Wifi_Pw_Menu_Enter(Key_Pressed_t key){
    uint8_t pw[WIFI_PW_ADDR_SIZE];
    uint8_t res=1;
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        res = confd.read_wifi_pw(pw);
        xSemaphoreGive(xMutexI2c);
    }
    if(res != 0){
        Log.error("Failed to read wifi pw" CR);
        edit_menu(pw, WIFI_PW_ADDR_SIZE, standard_alphabet, standard_alphabet_size, true);
    }
    else {
        edit_menu(pw, WIFI_PW_ADDR_SIZE, standard_alphabet, standard_alphabet_size, false);
    }
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        res = confd.store_wifi_pw(pw);
        xSemaphoreGive(xMutexI2c);
    }
    if (res != 0){
        lcd_buffer.print(0, "Failed!!!");
        Log.error("Failed to save wifi pw" CR);
        vTaskDelay(2000);
    }
    Menu_Navigate(&Menu_1_2);
}

static void Wifi_Mode_Menu_Select(int parent_index){
    WifiMode mode;
    uint8_t res;
    const char * mode_str[] = {"", "STA", "AT"};
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        res = confd.read_wifi_mode(&mode);
        xSemaphoreGive(xMutexI2c);
    }
    if(res != 0){
        Log.error("Failed to read wifi mode" CR);
        lcd_buffer.print(1, "N/A");
    }
    else {
        lcd_buffer.print(1, mode_str[mode.mode]);
    }
}

static void Wifi_Mode_Menu_Enter(Key_Pressed_t key){
    uint8_t res=1;
    WifiMode mode;
    const char * mode_str[] = {"", "STA", "AT"};
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        res = confd.read_wifi_mode(&mode);
        xSemaphoreGive(xMutexI2c);
    }
    if(res != 0){
        Log.error("Failed to read wifi mode" CR);
        //edit_menu(pw, WIFI_PW_ADDR_SIZE, standard_alphabet, standard_alphabet_size, true);
    }
    /*
    else {
        edit_menu(pw, WIFI_PW_ADDR_SIZE, standard_alphabet, standard_alphabet_size, false);
    }
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        res = confd.store_wifi_pw(pw);
        xSemaphoreGive(xMutexI2c);
    }
    if (res != 0){
        lcd_buffer.print(0, "Failed!!!");
        Log.error("Failed to save wifi pw" CR);
        vTaskDelay(2000);
    }
    */
    Menu_Navigate(&Menu_1_2);
}


void edit_menu(uint8_t *data, int len, const char *alphabet, int alphabet_len, bool do_reset_data){
    EncActions enc_action;
    int pos=0;
    int pos_old=0;
    int symb=0;
    int symb_old=0;

    if (do_reset_data) for(int i=0; i<len; i++)data[i]=' ';

    enum Modes {
        edit_mode=0,
        moving_mode
    };
    Modes mode = edit_mode;
    Modes mode_old = mode;

    lcd_buffer.printarray(1, data, len);
    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
        lcd_buffer.cursor_pos(pos, 1);
        lcd_buffer.blink_cursor_on_off(true);
        lcd_buffer.underscore_cursor_on_off(true);
        xSemaphoreGive(xMutexI2c);
    }
    while(1){
        if (uxQueueMessagesWaiting(encActionsQueue) > 0){
            xQueueReceive(encActionsQueue, &enc_action, portMAX_DELAY);
            switch(enc_action){
                case encActionCwMove:
                    if (mode == moving_mode) if (pos < len-1) pos++;
                    if (mode == edit_mode) if (symb < alphabet_len-1) symb++;
                break;
                case encActionCcwMove:
                    if (mode == moving_mode) if (pos > 0) pos--;
                    if (mode == edit_mode) if (symb > 0) symb--;
                break;
                case encActionBtnPressed:
                    if (mode == edit_mode){
                        mode = moving_mode;
                        if (pos < len-1) pos++;
                    }
                    else {
                        mode = edit_mode;
                        symb = 0;
                    }
                break;
                case encActionBtnLongPressed:
                    if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
                        lcd_buffer.blink_cursor_on_off(false);
                        lcd_buffer.underscore_cursor_on_off(false);
                        xSemaphoreGive(xMutexI2c);
                    }
                    return;
                break;
            }
        }
        if (symb != symb_old || pos != pos_old || mode != mode_old){
            if (xSemaphoreTake(xMutexI2c, portMAX_DELAY) == pdTRUE){
                if (symb != symb_old){
                    symb_old = symb;
                    data[pos] = alphabet[symb];
                    lcd_buffer.printsymb(pos, 1, alphabet[symb]);
                }
                if (pos != pos_old){
                    pos_old = pos;
                    lcd_buffer.cursor_pos(pos, 1);
                }
                if (mode != mode_old){
                    mode_old = mode;
                    if (mode == edit_mode) lcd_buffer.blink_cursor_on_off(true);
                    if (mode == moving_mode) lcd_buffer.blink_cursor_on_off(false);
                }
                xSemaphoreGive(xMutexI2c);
            }
        }
        vTaskDelay(10);
    }
}
