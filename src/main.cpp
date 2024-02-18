#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SPI.h>
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

void timinit();

#define FRONT_LIGHT_PIN 13

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

bool light1Bool = false;

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
  SPIFFS.begin();
  lv_fs_stdio_init();

  /* Configuring GPIO*/
  pinMode(FRONT_LIGHT_PIN, OUTPUT);

  /* Final settings*/
  ui_init();
  lv_obj_add_event_cb(ui_Light_Switch_1, change_light1, LV_EVENT_ALL, NULL);
}

void loop() {
  lv_timer_handler();
  delay(5);
}

static void change_light1(lv_event_t * e){
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_VALUE_CHANGED){
    light1Bool = !light1Bool;
    digitalWrite(FRONT_LIGHT_PIN, light1Bool);
  }
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
