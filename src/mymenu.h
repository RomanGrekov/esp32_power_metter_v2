#ifndef MYMENU
#define MYMENU

#include "mymicromenu.h"

/*
    MENU callbacks prototypes
*/
static void Level1Item1_Enter(Key_Pressed_t key);
static void Level1Item1_Select(int parent_index);

static void Read_Sensors_Enter(Key_Pressed_t key);
static void sensor_enter(Key_Pressed_t key);
static void sensor_select(int parent_index);

static void sensor_add(Key_Pressed_t key);
static void sensor_del(Key_Pressed_t key);
static void sensor_show(Key_Pressed_t key);
static void sensor_add_select(int parent_index);
static void showAllSensorsRuntime(Key_Pressed_t key);

static void Wifi_Name_Menu_Enter(Key_Pressed_t key);
static void Wifi_Name_Menu_Select(int parent_index);
static void Wifi_Pw_Menu_Enter(Key_Pressed_t key);
static void Wifi_Pw_Menu_Select(int parent_index);
static void Wifi_Mode_Menu_Enter(Key_Pressed_t key);
static void Wifi_Mode_Menu_Select(int parent_index);
/*
    LCD print callback
*/
static void Generic_Write(const char* Text);

static void enter_uppermenu(Key_Pressed_t key){
	Menu_Navigate(MENU_PARENT);
}

static void select_uppermenu(int parent_index){
}

//        NAME     NEXT,    PREV    PARENT,    CHILD      SelectCallback        EnterCallback  Text
MENU_ITEM(Menu_1, Menu_2, Menu_3, NULL_MENU, NULL_MENU, NULL              , showAllSensorsRuntime , "Show sensors", 0);
MENU_ITEM(Menu_2, Menu_3, Menu_1, NULL_MENU, Menu_1_1 , Level1Item1_Select, Level1Item1_Enter, "Settings", 0);
MENU_ITEM(Menu_3, Menu_1, Menu_2, NULL_MENU, NULL_MENU, NULL              , NULL             , "Menu 3", 0);

MENU_ITEM(Menu_1_1, Menu_1_2, UpperMenu1, NULL_MENU, Menu_2_1, NULL, NULL, "Sensors", 0);
MENU_ITEM(Menu_1_2, UpperMenu1, Menu_1_1, NULL_MENU, Menu_1_2_1, NULL, NULL, "WiFi", 0);
MENU_ITEM(UpperMenu1, Menu_1_1, Menu_1_2, Menu_1, NULL_MENU, select_uppermenu, enter_uppermenu, "Exit?", 0);

MENU_ITEM(Menu_2_1, Menu_2_2, UpperMenu2, NULL_MENU, Sensor_1, NULL, Read_Sensors_Enter, "Show existing", 0);
MENU_ITEM(Menu_2_2, UpperMenu2, Menu_2_2, NULL_MENU, NULL_MENU, NULL, NULL, "Add sensor", 0);
MENU_ITEM(UpperMenu2, Menu_2_1, Menu_2_2, Menu_1_1, NULL_MENU, select_uppermenu, enter_uppermenu, "Exit?", 0);

MENU_ITEM(Menu_1_2_1, Menu_1_2_2, UpperMenu1_2, NULL_MENU, NULL_MENU, Wifi_Name_Menu_Select, Wifi_Name_Menu_Enter, "WiFi name", 0);
MENU_ITEM(Menu_1_2_2, Menu_1_2_3, Menu_1_2_1, NULL_MENU, NULL_MENU, Wifi_Pw_Menu_Select, Wifi_Pw_Menu_Enter, "WiFi password", 0);
MENU_ITEM(Menu_1_2_3, UpperMenu1_2, Menu_1_2_2, NULL_MENU, NULL_MENU, Wifi_Mode_Menu_Select, Wifi_Mode_Menu_Enter, "WiFi mode", 0);
MENU_ITEM(UpperMenu1_2, Menu_1_2_1, Menu_1_2_3, Menu_2, NULL_MENU, select_uppermenu, enter_uppermenu, "Exit?", 0);

MENU_ITEM(Sensor_1, Sensor_2,    UpperMenu3, NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 1", 0);
MENU_ITEM(Sensor_2, Sensor_3,    Sensor_1,   NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 2", 1);
MENU_ITEM(Sensor_3, Sensor_4,    Sensor_2,   NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 3", 2);
MENU_ITEM(Sensor_4, Sensor_5,    Sensor_3,   NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 4", 3);
MENU_ITEM(Sensor_5, Sensor_6,    Sensor_4,   NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 5", 4);
MENU_ITEM(Sensor_6, Sensor_7,    Sensor_5,   NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 6", 5);
MENU_ITEM(Sensor_7, Sensor_8,    Sensor_6,   NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 7", 6);
MENU_ITEM(Sensor_8, Sensor_9,    Sensor_7,   NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 8", 7);
MENU_ITEM(Sensor_9, Sensor_10,   Sensor_8,   NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 9", 8);
MENU_ITEM(Sensor_10, UpperMenu3, Sensor_9,   NULL_MENU, Add_sensor, sensor_select, sensor_enter, "Sensor 10", 9);
MENU_ITEM(UpperMenu3, Sensor_1, Sensor_10, Menu_1_1, NULL_MENU, select_uppermenu, enter_uppermenu, "Exit?", 0);

MENU_ITEM(Add_sensor, Del_sensor, UpperMenu4, NULL_MENU, NULL_MENU, sensor_add_select, sensor_add, "Add", 0);
MENU_ITEM(Del_sensor, Show_sensor, Add_sensor, NULL_MENU, NULL_MENU, NULL, sensor_del, "Del", 0);
MENU_ITEM(Show_sensor, UpperMenu4, Del_sensor, NULL_MENU, NULL_MENU, NULL, sensor_show, "Check", 0);
MENU_ITEM(UpperMenu4, Add_sensor, Show_sensor, Menu_2_1, NULL_MENU, select_uppermenu, enter_uppermenu, "Exit?", 0);
#endif
