#include <Arduino.h>
#include "FS.h"
#include "SPIFFS.h"
#include <SPI.h>
#include <EEPROM.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <lvgl.h>
#include "gui/ui.h"

static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

void IRAM_ATTR onTimer();
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
static void change_light1(lv_event_t * e);
void sens();
void loadIcons();

void timinit();

#define FRONT_LIGHT_PIN 13
#define HALL_SENSOR_PIN 12
#define WHEEL_LENGTH 2

#define I2S_DOUT      22
#define I2S_BCLK      26
#define I2S_LRC       25

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

bool light1Bool = false;
float DIST = 0;
float SPEED = 0;
unsigned long lastturn;

void setup() {
  Serial.begin(115200);
  lv_init();
  timinit();

  // Initialise the TFT screen
  tft.init();
  tft.setRotation(1);
  uint16_t calData[5] = {277, 3440, 435, 3445, 6};
  tft.setTouch(calData);

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  /* Initialize spiffs file system*/
  if(!SPIFFS.begin())
    {
      Serial.println("Error accessing microSD card!");
      while(true); 
    }
  lv_fs_stdio_init();
  DIST=(float)EEPROM.read(0)/10.0;

  /* Configuring GPIO*/
  pinMode(FRONT_LIGHT_PIN, OUTPUT);
  attachInterrupt(HALL_SENSOR_PIN, sens, RISING);

  /* Final settings*/
  ui_init();
  loadIcons();
  lv_obj_add_event_cb(ui_Main_Light1_Swt, change_light1, LV_EVENT_ALL, NULL);
  //lv_arc_set_value(ui_Speedometer_Arc, 15);
}

void loop() { 
  lv_timer_handler();
  lv_obj_t* currentScreen = lv_scr_act();
  if ((millis()-lastturn)>2000){ //если сигнала нет больше 2 секунды
    SPEED=0;  //считаем что SPEED 0
    EEPROM.write(0,(float)DIST*10.0); //записываем DIST во внутреннюю память. Сделал так хитро, потому что внутренняя память не любит частой перезаписи. Также *10, чтобы сохранить десятую долю
  }
  if(currentScreen == ui_MainApp){
    lv_arc_set_value(ui_Main_Speed_Arc, (int)floor(SPEED));
    lv_label_set_text(ui_Main_Speed_Label, String((int)floor(SPEED)).c_str());
  }
  if(currentScreen == ui_RoadApp){
    lv_arc_set_value(ui_Road_Speed_Arc, (int)floor(SPEED));
    lv_label_set_text(ui_Road_Speed_Label, String((int)floor(SPEED)).c_str());
    lv_label_set_text(ui_Road_Dist_Value, (String((int)floor(DIST)) + " km").c_str());
  }
  delay(5);
}

static void change_light1(lv_event_t * e){
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_VALUE_CHANGED){
    light1Bool = !light1Bool;
    digitalWrite(FRONT_LIGHT_PIN, light1Bool);
  }
}

void sens() {
  if (millis()-lastturn > 80) {  //защита от случайных измерений (основано на том, что велосипед не будет ехать быстрее 120 кмч)
    SPEED = WHEEL_LENGTH / ((float)(millis() - lastturn) / 1000) * 3.6;  //расчет скорости, км/ч
    lastturn = millis();  //запомнить время последнего оборота
    DIST = DIST + WHEEL_LENGTH / 1000;  //прибавляем длину колеса к дистанции при каждом обороте оного
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
