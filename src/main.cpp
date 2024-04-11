#include <Arduino.h>
#include "FS.h"
#include "SPIFFS.h"
#include <SPI.h>
#include <EEPROM.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <lvgl.h>
#include "gui/ui.h"
#include <Preferences.h>
#include "bitmap.h"

static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

void IRAM_ATTR onTimer();
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void handle_apps();
static void change_light1(lv_event_t * e);
static void start_countdown(lv_event_t * e);
static void clear_dist(lv_event_t * e);
static void change_debug(lv_event_t * e);
void sens();
void loadIcons();
void timinit();

#define FRONT_LIGHT_PIN 13
#define HALL_SENSOR_PIN 12

#define I2S_DOUT      22
#define I2S_BCLK      26
#define I2S_LRC       25

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
Preferences prefs;

bool light1Bool = false;
bool debug = false;
float WHEEL_LEN = 2.007; /* Wheel length in meters, without a dot (x1000)*/
float DIST = 0;
float SPEED = 0;

/* Timer variables*/
unsigned long lastturn;
unsigned long lastchange;
bool startcounter = false;
int8_t clearCounter = 0;

void setup() {
  Serial.begin(115200);
  prefs.begin("lvgl-bike", false);
  debug = prefs.getBool("debug", false);
  lv_init();
  timinit();

  // Initialise the TFT screen
  tft.init();
  tft.setRotation(1);
  uint16_t calData[5] = {277, 3440, 435, 3445, 6};
  tft.setTouch(calData);
      if(debug)tft.setTextSize(2);tft.setTextColor(ILI9341_RED);tft.println("LVGL-BIKE");
      if(debug)tft.drawBitmap(0,0,bmp_lvgl_bike,128,64,ILI9341_WHITE);
      if(debug)tft.setTextColor(ILI9341_WHITE);
      if(debug)tft.setTextSize(1);
      if(debug)tft.fillScreen(ILI9341_BLACK);
      if(debug)tft.println("[INIT] Started display.");

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);

      if(debug)tft.println("[INIT] Initialized display driver.");
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
      if(debug)tft.println("[INIT] Registered display driver.");

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
      if(debug)tft.println("[INIT] Registered touch driver.");

  /* Initialize spiffs file system*/
  if(!SPIFFS.begin())
    {
      Serial.println("Error accessing microSD card!");
          if(debug)tft.println("[ERROR] Error accessing SPIFFS filesystem!");
      while(true); 
    }
      if(debug)tft.println("[INIT] Initialized SPIFFS filesystem.");
  //lv_fs_stdio_init();
  DIST=prefs.getFloat("dist", 0.0);
  //DIST=(float)EEPROM.read(0)/10.0;
  WHEEL_LEN=prefs.getFloat("wh-len", 2);
  //WHEEL_LEN=(float)EEPROM.read(10)/1000;
      if(debug)tft.println("[INIT] Loaded road vaiables.");
  /* Configuring GPIO*/
  pinMode(FRONT_LIGHT_PIN, OUTPUT);
  attachInterrupt(HALL_SENSOR_PIN, sens, RISING);
      if(debug)tft.println("[INIT] Initialized GPIO.");
      if(debug)tft.println('\n');
  /* Final settings*/
      if(debug)tft.println("[CLEAN] All done. Starting ui...");
      if(debug)delay(5000);
  ui_init();
  loadIcons();
  switch (debug)
  {
  case true:
    lv_obj_add_state(ui_Set_Entry_Debug_Swt, LV_STATE_CHECKED);
    break;
  case false:
    lv_obj_clear_state(ui_Set_Entry_Debug_Swt, LV_STATE_CHECKED);
  default:
    break;
  }
  lv_obj_add_event_cb(ui_Main_Light1_Swt, change_light1, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_Set_Entry_SetWheelLen_Spin, start_countdown, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_Set_Entry_ClearDist_Btn, clear_dist, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_Set_Entry_Debug_Swt, change_debug, LV_EVENT_ALL, NULL);
  lv_spinbox_set_value(ui_Set_Entry_SetWheelLen_Spin, WHEEL_LEN*1000);
}

void loop() { 
  lv_timer_handler();
  if ((millis()-lastturn)>2000){ //если сигнала нет больше 2 секунды
    SPEED=0;  //считаем что SPEED 0
    prefs.putFloat("dist", DIST); //записываем DIST во внутреннюю память. Сделал так хитро, потому что внутренняя память не любит частой перезаписи. Также *10, чтобы сохранить десятую долю
  }
  handle_apps();
  delay(5);
}

void handle_apps(){
  lv_obj_t* currentScreen = lv_scr_act();
  /* MAIN */
  if(currentScreen == ui_MainApp){
    lv_arc_set_value(ui_Main_Speed_Arc, (int)floor(SPEED));
    lv_label_set_text(ui_Main_Speed_Label, String((int)floor(SPEED)).c_str());
  }
  /* ROAD */
  if(currentScreen == ui_RoadApp){
    lv_arc_set_value(ui_Road_Speed_Arc, (int)floor(SPEED));
    lv_label_set_text(ui_Road_Speed_Label, String((int)floor(SPEED)).c_str());
    lv_label_set_text(ui_Road_Dist_Value, (String((int)floor(DIST)) + " km").c_str());
  }
  /* SETTINGS */
  if(currentScreen == ui_SettingsApp){
    if(startcounter){
      if((millis()-lastchange)>2000){
        lastchange = millis();
        prefs.putFloat("wh-len", WHEEL_LEN);
        startcounter = false;
      }
    }
    switch (clearCounter)
    {
    case 1:
      lv_label_set_text(ui_Set_Entry_ClearDist_Label, "Are you sure?");
      break;
    case 2:
      lv_label_set_text(ui_Set_Entry_ClearDist_Label, "This will clear your dist!");
      break;
    case 3:
      lv_label_set_text(ui_Set_Entry_ClearDist_Label, "Last chance!");
      break;
    case 4:
      lv_label_set_text(ui_Set_Entry_ClearDist_Label, "Done. Distance: 0 km.");
      prefs.putFloat("dist", 0.0);
      DIST = prefs.getFloat("dist", 0.0);
    default:
      break;
    }
    if(clearCounter > 4 or clearCounter < 0){
      lv_label_set_text(ui_Set_Entry_ClearDist_Label, "Clear distance");
    }
  }
}

static void change_light1(lv_event_t * e){
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_VALUE_CHANGED){
    light1Bool = !light1Bool;
    digitalWrite(FRONT_LIGHT_PIN, light1Bool);
  }
}

static void start_countdown(lv_event_t * e){
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_VALUE_CHANGED){
    WHEEL_LEN = (float)lv_spinbox_get_value(ui_Set_Entry_SetWheelLen_Spin) / 1000;
    startcounter = true;
  }
}
static void clear_dist(lv_event_t * e){
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_CLICKED){
    clearCounter++;
  }
}
static void change_debug(lv_event_t * e){
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_VALUE_CHANGED){
    prefs.putBool("debug", !debug);
    debug = prefs.getBool("debug", false);
  }
}

void sens() {
  if (millis()-lastturn > 80) {  //защита от случайных измерений (основано на том, что велосипед не будет ехать быстрее 120 кмч)
    SPEED = WHEEL_LEN / ((float)(millis() - lastturn) / 1000) * 3.6;  //расчет скорости, км/ч
    lastturn = millis();  //запомнить время последнего оборота
    DIST = DIST + WHEEL_LEN / 1000;  //прибавляем длину колеса к дистанции при каждом обороте оного
  }
}

void loadIcons(){
  lv_label_set_text(ui_Main_Charge_Icon,LV_SYMBOL_CHARGE);
  lv_label_set_text(ui_Main_RoadApp_Icon,LV_SYMBOL_HOME);
  lv_label_set_text(ui_Main_SetApp_Icon,LV_SYMBOL_SETTINGS);
  lv_label_set_text(ui_Main_NaviApp_Icon,LV_SYMBOL_GPS);
  lv_label_set_text(ui_Main_ClockApp_Icon,LV_SYMBOL_BELL);
  lv_label_set_text(ui_Road_Back_Label,LV_SYMBOL_LEFT);
  lv_label_set_text(ui_Set_TopBackBtn_Label,LV_SYMBOL_RIGHT);
  lv_label_set_text(ui_Set_TopIcon,LV_SYMBOL_SETTINGS);
}

void IRAM_ATTR onTimer()
{
  lv_tick_inc(10);
}

void timinit()
{

  static hw_timer_t *timer1 = NULL;
  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, onTimer, true);
  timerAlarmWrite(timer1, 10000, true);
  timerAlarmEnable(timer1);
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  uint16_t touchX, touchY;

  bool touched = tft.getTouch(&touchX, &touchY, 600);
  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;

    data->point.x = touchY; // Костыль, но лучше решения не нашел
    data->point.y = touchX;
  }
}
