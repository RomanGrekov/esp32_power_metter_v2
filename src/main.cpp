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
    encActionCcwMove
};

/*
    Current selected sensor number in lcd menu
*/
int current_sensor_n=0;

/*
  Mcp23017 connected leds
*/
McpLeds Leds(0, MAX_SENSORS_AMOUNT);

/*
  Do get data from sensors
*/
bool DO_READ_SENSORS=false;
/*
  Array of all sensors data
*/
Sensor sensors_data[MAX_SENSORS_AMOUNT];
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
    RTOS tasks prototypes
*/
void taskControlLed( void * parameter );
void taskLcd( void * parameter );
void taskEncoderRead( void * parameter);
void taskMenu( void * parameter );
void taskSensorsRead( void * parameter );
void taskDetectCharging( void * parameter );
void taskChargingLeds( void * parameter );
void taskChargingStats( void * parameter );

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
        Start leds
    */
    Leds.init();

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
                1000,
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
    bool old_btn_state=false;
    bool btn_state=false;
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
        btn_state = enc_btn.isPressed();
        if (btn_state != old_btn_state){
            Log.notice("Btn pressed: %d" CR, btn_state);
            old_btn_state = btn_state;
            if (btn_state == 0){
                enc_actions = encActionBtnPressed;
                xQueueSend(encActionsQueue, &enc_actions, portMAX_DELAY);
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

void taskLcd( void * parameter ) {
    /*
        Setup LCD
        MUST to be called here. Otherwise LCD gluchit
    */
    lcd.init();
    lcd.backlight();
    bool need_retry=false;

    while(1){
        if (lcd_buffer.is_changed() || need_retry){
            if (xSemaphoreTake(xMutexI2c, pdMS_TO_TICKS(100)) == pdTRUE){
                lcd_buffer.show();
                xSemaphoreGive(xMutexI2c);
                need_retry = false;
            }
            else{
                need_retry = true;
                vTaskDelay(10);
            }
        }
        vTaskDelay(100);
    }
}

void taskSensorsRead( void * parameter ) {
    uint8_t addr=0;
    int index_n=0;
    uint8_t indexes[MAX_SENSORS_AMOUNT];
    bool do_not_read_old=DO_READ_SENSORS;

    PZEM004Tv30 pzem(&Serial2, addr); // Fake init
    uint8_t ws_amount=0;
    while(1){
      if (DO_READ_SENSORS){
          while(ws_amount == 0){
              /*
                  Read sensors addresses from eeprom
              */
              Read_Sensors_Enter(KEY_OK);
              /*
                  Find working sensors
              */
              ws_amount = confd.get_sensors_indexes(indexes);

              if (ws_amount == 0 ){
                  Log.error( "No one working sensors to show" CR);
                  vTaskDelay(1000);
              }
              index_n=0;
          }
          addr = confd.get_address(indexes[index_n]);
          xSemaphoreTake(xMutexSensorRead, portMAX_DELAY);
          pzem.init(addr);
          sensors_data[indexes[index_n]].address = addr;
          sensors_data[indexes[index_n]].V = pzem.voltage();
          sensors_data[indexes[index_n]].A = pzem.current();
          sensors_data[indexes[index_n]].Kwh = pzem.energy();
          xSemaphoreGive(xMutexSensorRead);

          if (index_n < ws_amount-1) index_n++;
          else index_n = 0;
          if (do_not_read_old != DO_READ_SENSORS) do_not_read_old = DO_READ_SENSORS;
      }
      else {
          if (do_not_read_old != DO_READ_SENSORS){
              do_not_read_old = DO_READ_SENSORS;
              for(int i=0; i<MAX_SENSORS_AMOUNT; i++){
                  sensors_data[i].V=0;
                  sensors_data[i].A=0;
                  sensors_data[i].Kwh=0;
              }
          }
      }
      vTaskDelay(1000);
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
    bool done=false;

    while(1){
        done=false;
        while (!done){
            if (xSemaphoreTake(xMutexI2c, pdMS_TO_TICKS(100)) == pdTRUE){
                for (int id=0; id<MAX_SENSORS_AMOUNT; id++){
                    if (sensors_data[id].is_charging) Leds.on(id);
                    else Leds.off(id);
                }
                xSemaphoreGive(xMutexI2c);
                done = true;
            } else vTaskDelay(10);
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
    Log.notice("Reading sensors addresses");
    for (int i=0; i < MAX_SENSORS_AMOUNT; i++) confd.add_sensor(i, 0);
    bool done=false;
    while(!done){
        if (xSemaphoreTake(xMutexI2c, pdMS_TO_TICKS(100)) == pdTRUE){
            uint8_t res = confd.read_sensors();
            if (res != 0) Log.error("Failed to read sensor addresses" CR);
            xSemaphoreGive(xMutexI2c);
            done = true;
        }
        else vTaskDelay(10);
    }
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
    uint8_t dev_addr=0;

    lcd_buffer.print(0, "Searching sensors");

    /*
      Do not try to read data from sensors
    */
    DO_READ_SENSORS = false;
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
            bool done=false;
            while (!done){
                if (xSemaphoreTake(xMutexI2c, pdMS_TO_TICKS(100)) == pdTRUE){
                    res_int = confd.store_sensors();
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

    /*
      Do not try to read data from sensors
    */
    DO_READ_SENSORS = false;

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
                        confd.del_sensor(current_sensor_n);
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
    EncActions enc_action;
    DO_READ_SENSORS = true;
    uint8_t addr = confd.get_address(current_sensor_n);
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

    Read_Sensors_Enter(KEY_OK);

    DO_READ_SENSORS = true;

    index_n=0;
    while(1){
        if (ws_amount == 0 ){
            ws_amount = confd.get_sensors_indexes(indexes);
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
