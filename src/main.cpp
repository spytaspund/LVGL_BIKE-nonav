#include <Arduino.h>
#include "FS.h"
#include "SPIFFS.h"
#include <SPI.h>
#include <EEPROM.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <lvgl.h>
#include "gui/ui.h"
#include <Preferences.h>
#include <RtcDS1302.h>
#include "bitmap.h"
#include <MFRC522.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "battery.h"

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
static void change_clock(lv_event_t * e);
String formatTime(int num);
String getDOW(int8_t dow);
void grab_gps();
void displayInfo();
void sens();
void loadIcons();
void timinit();

#define FRONT_LIGHT_PIN   12
#define HALL_SENSOR_PIN   35

#define I2S_DOUT          22
#define I2S_BCLK          26
#define I2S_LRC           25

#define RTC_DAT           26
#define RTC_SCLK          27
#define RTC_RST           25

#define RFID_RST          16
#define RFID_CS           17

#define GPS_TX            22
#define GPS_RX            13

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
Preferences prefs;
ThreeWire myWire(RTC_DAT,RTC_SCLK,RTC_RST);
RtcDS1302<ThreeWire> Rtc(myWire);
MFRC522 rfid(RFID_CS, RFID_RST);
TinyGPSPlus tgps;
SoftwareSerial gps(GPS_RX, GPS_TX); // 2 to Tx, 3 unconnected
Battery internal_batt(36);
Battery external_batt(39);

bool light1Bool = false;
bool debug = false;
int8_t clockMode = 0;
float WHEEL_LEN = 2.007; /* Wheel length in meters, without a dot (x1000)*/
float DIST = 0;
float SPEED = 0;

/* Timer variables*/
unsigned long update_hall;
unsigned long update_settings;
unsigned long update_road;
unsigned long update_clock;
unsigned long gps_record;
unsigned long minute_timer;
bool startcounter = false;
bool recordgps = true;
int8_t clearCounter = 0;

void setup() {
  Serial.begin(115200);
  gps.begin(9600);
  prefs.begin("lvgl-bike", false);
  debug = prefs.getBool("debug", false);
  lv_init();
  timinit();

  /* Initializing TFT screen... */
  tft.init();
  tft.setRotation(1);
  uint16_t calData[5] = {277, 3440, 435, 3445, 6};
  tft.setTouch(calData);
      if(debug)tft.fillScreen(ILI9341_BLACK);
      if(debug)tft.setTextColor(ILI9341_WHITE);
      if(debug)tft.setTextSize(1);
      if(debug)tft.println("Welcome to");
      if(debug)tft.drawBitmap(0,8,bmp_lvgl_bike,128,64,ILI9341_RED);
      if(debug)tft.setCursor(0, 74);
      if(debug)delay(500);
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
          if(debug)tft.println("[ERROR] Error accessing SPIFFS filesystem!");
      while(true); 
    }
      if(debug)tft.println("[INIT] Initialized SPIFFS filesystem.");
  //lv_fs_stdio_init();
  DIST=prefs.getFloat("dist", 0.0);
  WHEEL_LEN=prefs.getFloat("wh-len", 2);
      if(debug)tft.println("[INIT] Loaded road vaiables.");

  /* Configuring GPIO*/

  pinMode(FRONT_LIGHT_PIN, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);
  //attachInterrupt(HALL_SENSOR_PIN, sens, RISING);     //it will be used later in loop, attachInterrupt causes some kernel panics
      if(debug)tft.println("[INIT] Initialized GPIO.");
      if(debug)tft.println('\n');
  
  /* Configuring RTC clock*/

  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  if(debug)tft.println("[INIT] Initializing RTC DS1302.");
  if(debug)tft.println("[RTC] Compiled time: " + String(__DATE__) + ", " + String(__TIME__));

  /* Checking ALL THE ERRORS (idk why this is so important)*/
  
  if (!Rtc.IsDateTimeValid()) {
        // Common Causes:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing
          if(debug)tft.println("[ERROR] RTC lost confidence in the DateTime!");
      Rtc.SetDateTime(compiled);
  }
  if (Rtc.GetIsWriteProtected()){
          if(debug)tft.println("[WARN] RTC was write protected, bypassing...");
      Rtc.SetIsWriteProtected(false);
  }
  if (!Rtc.GetIsRunning()){
          if(debug)tft.println("[WARN] RTC wasn't actively running, starting...");
      Rtc.SetIsRunning(true);
  }
  if (Rtc.GetDateTime() < compiled) {
          if(debug)tft.println("[ERROR] RTC is older that actual time! Updating...");
      Rtc.SetDateTime(compiled);
  }
  RtcDateTime now = Rtc.GetDateTime();
  if(debug)tft.println("[RTC] Module time: \n - Date: " + String(now.Day()) + "." + String(now.Month()) + "." + String(now.Year()) + "\n - Time: " + String(now.Hour()) + ":" + String(now.Minute()) + ":" + String(now.Second()));
  if(debug)tft.println("[INIT] RTC successfully initialized.");

  /* RFID */
  SPI.begin();
  rfid.PCD_Init();

  /* Final settings*/
      if(debug)tft.println("[CLEAN] All done. Starting ui...");
      if(debug)delay(5000);
  ui_init();
  loadIcons();
  if(debug) lv_obj_add_state(ui_Set_Entry_Debug_Swt, LV_STATE_CHECKED);
  if(!debug) lv_obj_clear_state(ui_Set_Entry_Debug_Swt, LV_STATE_CHECKED);
  lv_obj_add_event_cb(ui_Main_Light1_Swt, change_light1, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_Set_Entry_SetWheelLen_Spin, start_countdown, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_Set_Entry_ClearDist_Btn, clear_dist, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_Set_Entry_Debug_Swt, change_debug, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_Set_Entry_Debug_Swt, change_clock, LV_EVENT_ALL, NULL);
  lv_spinbox_set_value(ui_Set_Entry_SetWheelLen_Spin, WHEEL_LEN*1000);
}

void loop() { 
  grab_gps();
  lv_timer_handler();
  grab_gps();
  if(1700 > analogRead(35) or analogRead(35) > 2000) sens();
  grab_gps();
  if(recordgps){
    if(tgps.satellites.value() == 0){
      gps_record = millis();
    }
    if(tgps.satellites.value() > 0){
      gps_record = millis();
      recordgps = false;
    }
  }
  grab_gps();
  if ((millis()-update_hall)>2000){ //if there is no signal for 2 seconds
    SPEED=0;
    prefs.putFloat("dist", DIST); //записываем DIST во внутреннюю память. Сделал так хитро, потому что внутренняя память не любит частой перезаписи. Также *10, чтобы сохранить десятую долю
  }
  grab_gps();
  handle_apps();
  grab_gps();
}
void grab_gps(){
  if (gps.available()) {
    char c = gps.read();
    //Serial.print(c);
    tgps.encode(c);
  }
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (tgps.location.isValid())
  {
    Serial.print(tgps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(tgps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (tgps.date.isValid())
  {
    Serial.print(tgps.date.month());
    Serial.print(F("/"));
    Serial.print(tgps.date.day());
    Serial.print(F("/"));
    Serial.print(tgps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (tgps.time.isValid())
  {
    if (tgps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(tgps.time.hour());
    Serial.print(F(":"));
    if (tgps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(tgps.time.minute());
    Serial.print(F(":"));
    if (tgps.time.second() < 10) Serial.print(F("0"));
    Serial.print(tgps.time.second());
    Serial.print(F("."));
    if (tgps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(tgps.time.centisecond());
  }
  if(tgps.satellites.isValid()){
    Serial.print(tgps.satellites.value());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void handle_apps(){
  lv_obj_t* currentScreen = lv_scr_act();
  /* MAIN */
  if(currentScreen == ui_MainApp){
    RtcDateTime now = Rtc.GetDateTime();
    lv_arc_set_value(ui_Main_Speed_Arc, (int)floor(SPEED));
    lv_label_set_text(ui_Main_Speed_Label, String((int)floor(SPEED)).c_str());
    lv_label_set_text(ui_Main_Clock_Label, (String(now.Hour()) + "\n" + String(formatTime(now.Minute()))).c_str());
    lv_label_set_text(ui_Main_Clock_DOW, getDOW(now.DayOfWeek()).c_str());
    if((millis() - minute_timer)>60000){
      lv_label_set_text(ui_Main_Bat_Lv, String(internal_batt.getBatteryChargeLevel()).c_str());
      lv_bar_set_value(ui_Main_Charge_Prog, internal_batt.getBatteryChargeLevel(), LV_ANIM_ON);
    }
  }
  grab_gps();
  /* ROAD */
  if(currentScreen == ui_RoadApp){
    RtcDateTime now = Rtc.GetDateTime();
    lv_arc_set_value(ui_Road_Speed_Arc, (int)floor(SPEED));
    lv_label_set_text(ui_Road_Speed_Label, String((int)floor(SPEED)).c_str());
    lv_label_set_text(ui_Road_Dist_Value, (String((int)floor(DIST)) + " km").c_str());
    lv_label_set_text(ui_Road_Clock_Hours, String(now.Hour()).c_str());
    lv_label_set_text(ui_Road_Clock_Minutes, formatTime(now.Minute()).c_str());
  }
  grab_gps();
  /* SETTINGS */
  if(currentScreen == ui_SettingsApp){
    if(startcounter){
      if((millis()-update_settings)>2000){
        update_settings = millis();
        prefs.putFloat("wh-len", WHEEL_LEN);
        startcounter = false;
      }
    }
    /*                          no elseif? 
    ⣿⣿⣿⣿⢁⢂⢂⠢⢂⠢⡂⠆⡂⡢⢂⠢⢂⠢⢂⠢⢂⠢⢂⠢⢂⠢⢂⢂⠢⢂⠢⢂⠢⢂⠢⢂⠢⢂⠢⢂⠢⢂⠢⢂⠢⡂⡢⠡
    ⣿⣿⣿⡏⡢⠡⡂⠕⡐⢅⠢⢑⢐⠄⠕⡈⡢⢑⢐⠅⡢⢑⢐⠡⠡⡊⠔⡐⢅⠢⠡⠡⢊⠄⠕⡐⠡⡁⡪⠠⡑⡐⠅⢅⠊⢔⠨⢌
    ⣿⣿⣿⢇⠢⡑⢌⠌⡌⡂⣊⠢⡑⠌⢌⢂⠢⡑⠄⢕⢐⠅⡢⠡⡑⡐⢅⢊⠄⡅⢕⠡⡑⡨⡈⡢⢑⢐⢐⠅⡂⡊⢌⠢⠡⡑⢌⢆
    ⣿⣿⣿⢐⠅⡊⡢⢑⢔⠱⡐⢕⠨⡘⡐⢅⢑⠌⢜⢐⠅⡪⢐⢑⠌⢌⢂⠢⡑⢌⠢⡑⡐⢔⢐⠌⡂⡊⢔⠨⡐⢌⢂⠪⠨⡊⢆⠆
    ⣿⣿⡇⡢⡑⢅⢊⢆⢕⢅⢕⠡⡑⢌⢌⠢⡑⢌⠢⡢⡑⢌⢢⠡⡑⡑⢌⢌⢌⠢⡑⠔⢌⠢⠢⡑⢌⢂⠅⡊⠔⡡⢂⢕⢑⢜⠰⡑
    ⣿⣿⢑⠔⢌⢪⢸⠰⡑⢔⢐⢑⢌⢢⢡⠱⡨⠪⡨⠢⡊⡢⡡⡑⡑⡌⠆⢕⢐⠕⢌⢊⠢⡑⡑⢌⢂⠢⡑⠌⡌⠢⡑⢔⢱⢐⠕⡌
    ⣿⣿⢐⢅⢣⢱⢑⠕⡡⡑⢌⠢⠢⡑⢔⢱⠨⡪⡘⢌⡊⢆⠪⡨⠢⡊⡪⡨⢢⢑⢅⠕⢅⠕⢌⠢⡡⡑⢌⢌⢌⠪⡨⢢⢑⢅⠕⡌
    ⣿⣿⠢⡣⠣⡱⡐⡱⢐⢌⠢⠡⡃⡪⡘⢔⢱⠨⡊⢆⠪⡪⡘⡌⡪⡸⡐⢕⢱⢨⢢⢑⢅⠕⢅⠕⠔⡌⢆⠢⡢⠱⡨⡢⡱⡘⡌⡪
    ⣿⠏⢕⢘⢌⠢⡊⢔⠡⠢⡑⠡⢂⢆⢊⠪⡂⢇⠕⡅⡣⡊⢆⠪⡢⠪⡘⢌⢪⠢⡃⡇⡕⡕⡅⡣⡱⢨⠢⡱⡘⡌⡒⡌⡆⡣⡊⢆
    ⡿⢘⢌⠢⠢⡑⠌⡂⠅⠡⠄⡁⡂⡢⢡⠑⠸⡐⡕⢌⠆⡕⢅⠣⠪⡘⢌⠪⡂⡣⡱⡑⡕⡜⢜⢌⢎⢆⢣⢱⢨⢪⠸⡨⡪⡸⢨⠢
    ⣧⠂⢀⠁⡁⠄⠁⠄⡀⢂⢐⢐⠔⢌⠢⡑⡈⠨⠨⡂⢇⠪⡂⣃⠣⡑⢅⠣⡑⡌⢆⠕⡌⡪⡊⡎⢆⢣⢱⢑⢕⢜⢜⢌⢆⠕⡅⡣
    ⣿⣄⠄⢀⠄⠄⢄⠡⡐⡐⢅⠢⡑⢅⠪⡐⢌⠔⡀⠈⠄⡑⠌⡂⠕⠌⡂⠣⠑⢌⠢⡃⡊⡢⠱⡘⡜⢜⢜⢜⢜⢔⠕⡜⡰⡑⢌⢪
    ⣿⣿⣦⣐⠌⢌⠢⡑⡐⢅⠅⠕⢌⠢⡑⢌⠢⡑⢌⢂⢂⢀⠠⠄⠐⠄⠠⠈⠐⠄⠂⠠⠑⠨⠪⡘⢜⢜⢜⢜⢜⢔⢕⢑⢔⠸⡨⣢
    ⣿⣿⣿⣿⣷⡥⡑⠌⡌⠢⡑⢅⠕⡑⢌⠢⡑⢌⠢⡑⢔⠡⠢⡨⠠⢈⢀⠄⠈⡀⠈⠠⠈⠨⠐⢅⠕⡜⡜⡜⡜⢔⢑⠔⡂⡣⣵⣿
    ⣿⣿⣿⣿⣿⣿⣷⣕⠌⠌⡂⠅⡊⠔⡡⡑⢌⠢⠡⡊⠄⢕⠡⡊⢌⢂⠢⡡⡑⡐⢄⠐⡀⠂⠁⠢⡑⡕⡕⡕⢕⠅⢅⢅⣣⣾⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⣿⣷⡠⢄⠄⠄⠁⢂⠜⡐⠅⢕⠨⠨⡂⠅⢌⢂⠊⠌⢔⠰⠨⡂⢕⠐⢄⠡⢈⠢⡱⡱⣑⢑⠌⣲⣾⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⣿⡏⡎⡖⠄⠄⢔⠄⠨⡊⢌⢂⢊⢂⢂⠑⠄⠄⠁⠁⠂⠅⡑⢌⠢⡑⢅⠪⠠⡑⡕⡕⡢⡑⡛⢅⠆⣻⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⡿⠋⣎⢂⢀⠄⡈⠄⢨⠨⢂⢂⢂⠂⢄⢢⢣⠁⠄⠄⡀⠄⠨⠐⡨⢐⠔⠡⡑⢜⢜⠜⡀⡂⠌⣐⣴⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⢑⠅⡂⠅⢂⠠⠄⡂⢜⠌⠢⠨⠂⠌⡢⡣⡃⠄⠄⠥⠄⠄⢂⠄⢐⠡⡨⢂⢕⢕⠕⡐⡐⢁⣢⣾⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⡐⢕⢐⠨⡀⡂⡅⡪⢢⠡⡑⡡⠡⢁⠪⡪⡂⡀⡈⢀⠠⢐⢱⠐⡐⡕⡜⡔⡕⡕⡐⣐⣄⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⡏⡢⡡⠱⡨⡢⢣⠑⢅⢂⢂⠢⢁⠂⠄⡁⢃⠢⠐⡀⢐⢁⠅⢔⢱⢱⢕⢽⢸⣴⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⣿⢐⢌⢪⠢⡣⢡⠡⠡⢂⠢⡑⢄⢑⢐⠨⢐⠠⢁⠂⢅⠢⡑⡕⡕⡕⡝⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⣿⡇⡎⢆⠇⢕⢐⢅⢑⠔⡡⠨⡂⠢⡑⢌⢂⠪⡐⠅⡅⠣⡑⢌⠪⡊⣺⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⡟⡌⡎⡪⠪⡨⡢⡢⡱⡨⠢⡑⠌⢌⠢⠡⡂⡑⡈⡂⠪⠨⡨⠢⡑⣵⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⡿⡡⡣⢪⢘⢜⢜⢜⢜⢜⢜⠜⡌⡪⡂⢅⢑⠐⢌⢐⠌⢌⠪⡐⢅⣣⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⡟⡕⡜⡜⡜⡜⣜⢜⢮⢳⢱⢱⢱⠱⡨⢂⢂⠢⠡⡑⡐⢌⠢⡑⡜⣔⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⢧⢣⢣⢣⡣⡫⣎⢯⡪⡇⡇⢇⠣⡑⡐⡐⡐⠌⡂⡢⢊⢔⠱⣘⢬⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⠨⠪⡸⡸⡸⡹⡸⡸⡘⡌⡪⡢⡣⡪⡢⡪⢐⢁⠢⠨⡢⡊⡎⢦⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣏⠌⠔⢌⠪⡘⠌⠢⡱⡸⡸⢜⢎⢮⢪⢪⠢⡑⠨⡊⡢⡣⢣⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    ⣿⣿⣿⣿⣿⣿⣿⣿⡇⡊⡀⠡⢁⢂⢂⠎⡪⡲⡐⡅⡱⢱⠨⢂⢪⠸⣨⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
    */
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
  grab_gps();
  /* GPS */
  if(currentScreen == ui_GPSApp){
    String timeStr;
    lv_label_set_text(ui_GPS_Main_Time_Value, (formatTime(tgps.time.hour()) + ":" + formatTime(tgps.time.minute()) + ":" + formatTime(tgps.time.second())).c_str());
    lv_label_set_text(ui_GPS_Main_Sat_Value, String(tgps.satellites.value()).c_str());
    if(tgps.satellites.value() == 0){
      lv_obj_set_style_text_color(ui_GPS_Main_Sat_Value, lv_color_hex(0xD20000), 0); 
      lv_label_set_text(ui_GPS_Status_Label, "WARNING"); 
      lv_obj_set_style_text_color(ui_GPS_Status_Label, lv_color_hex(0xFFD400), 0); 
      lv_obj_set_style_text_color(ui_GPS_Status_Icon, lv_color_hex(0xFFD400), 0); 
      lv_label_set_text(ui_GPS_Status_Icon, LV_SYMBOL_CLOSE);
    }
    if(tgps.satellites.value() > 0){
      lv_obj_set_style_text_color(ui_GPS_Main_Sat_Value, lv_color_hex(0x000000), 0); 
      lv_label_set_text(ui_GPS_Status_Label, "ALL GREAT"); 
      lv_obj_set_style_text_color(ui_GPS_Status_Label, lv_color_hex(0x82CD47), 0); 
      lv_obj_set_style_text_color(ui_GPS_Status_Icon, lv_color_hex(0x82CD47), 0); 
      lv_label_set_text(ui_GPS_Status_Icon, LV_SYMBOL_OK);
    }
    lv_label_set_text(ui_GPS_Main_Record_Value, (String(gps_record / 1000 / 60) + " mins").c_str());
    lv_label_set_text(ui_GPS_Main_Lat_Value, String(tgps.location.lat(), 6).c_str());
    lv_label_set_text(ui_GPS_Main_Lon_Value, String(tgps.location.lng(), 6).c_str());
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
static void change_clock(lv_event_t * e){
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_CLICKED){
    clockMode++;
  }
}

void sens() {
  if (millis()-update_hall > 80) {  //защита от случайных измерений (основано на том, что велосипед не будет ехать быстрее 120 кмч)
    SPEED = WHEEL_LEN / ((float)(millis() - update_hall) / 1000) * 3.6;  //расчет скорости, км/ч
    update_hall = millis();  //запомнить время последнего оборота
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
  lv_label_set_text(ui_GPS_Main_Lat_Icon,LV_SYMBOL_GPS);
  lv_label_set_text(ui_GPS_Main_Lon_Icon,LV_SYMBOL_GPS);
  lv_label_set_text(ui_GPS_Main_Sat_Icon,LV_SYMBOL_WIFI);
  lv_label_set_text(ui_GPS_Main_Time_Icon,LV_SYMBOL_BELL);
  lv_label_set_text(ui_GPS_Back_Btn_Label,LV_SYMBOL_UP);
  lv_label_set_text(ui_Main_ClockApp_Icon,LV_SYMBOL_WIFI);
  lv_label_set_text(ui_GPS_Main_Record_Icon,LV_SYMBOL_BELL);
}

String formatTime(int num){
  if(num<10) return ("0" + String(num));
  if(num>=10) return String(num);
  return "228"; // This needs to calm the compiler, and this beautiful number will indicate the problem if it happens.
}

String getDOW(int8_t dow){
  switch (dow)
  {
  case 1:
    return "Ponedelnik";
    break;
  case 2:
    return "Vtornik";
    break;
  case 3:
    return "Sreda";
    break;
  case 4:
    return "Chetverg";
    break;
  case 5:
    return "Pyatnitsa";
    break;
  case 6:
    return "Subbota";
    break;
  case 0:
    return "Voskresenie";
    break;
  default:
    break;
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
