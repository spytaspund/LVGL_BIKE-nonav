// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: LVGL_BIKE

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_MainApp
void ui_MainApp_screen_init(void);
lv_obj_t * ui_MainApp;
lv_obj_t * ui_Main_Bat_bg;
lv_obj_t * ui_Main_Bat_Lv;
lv_obj_t * ui_Main_Charge_Icon;
lv_obj_t * ui_Main_Charge_Prog;
lv_obj_t * ui_Main_Light2;
lv_obj_t * ui_Main_Light2_Slider;
lv_obj_t * ui_Main_Light1;
lv_obj_t * ui_Main_Light1_Slider;
lv_obj_t * ui_Main_Light1_Swt;
lv_obj_t * ui_Main_Light2_Swt;
lv_obj_t * ui_Main_Speed;
lv_obj_t * ui_Main_Speed_Arc;
lv_obj_t * ui_Main_Speed_Label;
lv_obj_t * ui_Main_Dock;
void ui_event_Main_RoadApp(lv_event_t * e);
lv_obj_t * ui_Main_RoadApp;
lv_obj_t * ui_Main_RoadApp_Icon;
void ui_event_Main_SetApp(lv_event_t * e);
lv_obj_t * ui_Main_SetApp;
lv_obj_t * ui_Main_SetApp_Icon;
lv_obj_t * ui_Main_NaviApp;
lv_obj_t * ui_Main_NaviApp_Icon;
lv_obj_t * ui_Main_ClockApp;
lv_obj_t * ui_Main_ClockApp_Icon;


// SCREEN: ui_RoadApp
void ui_RoadApp_screen_init(void);
lv_obj_t * ui_RoadApp;
lv_obj_t * ui_Road_Speed;
lv_obj_t * ui_Road_Speed_Arc;
lv_obj_t * ui_Road_Speed_Label;
lv_obj_t * ui_Road_Kmh_Label;
lv_obj_t * ui_Road_Dist;
lv_obj_t * ui_Road_Dist_Label;
lv_obj_t * ui_Road_Dist_Value;
void ui_event_Road_Back_Btn(lv_event_t * e);
lv_obj_t * ui_Road_Back_Btn;
lv_obj_t * ui_Road_Back_Label;
lv_obj_t * ui_Road_Clock;
lv_obj_t * ui_Road_Clock_Hours;
lv_obj_t * ui_Road_Clock_Minutes;


// SCREEN: ui_SettingsApp
void ui_SettingsApp_screen_init(void);
lv_obj_t * ui_SettingsApp;
lv_obj_t * ui_Set_TopBar;
lv_obj_t * ui_Set_TopLabel;
lv_obj_t * ui_Set_TopIcon;
void ui_event_Set_TopBackBtn(lv_event_t * e);
lv_obj_t * ui_Set_TopBackBtn;
lv_obj_t * ui_Set_TopBackBtn_Label;
lv_obj_t * ui_Set_Main;
lv_obj_t * ui_Set_Entry_ClearDist;
lv_obj_t * ui_Set_Entry_ClearDist_Label;
lv_obj_t * ui_Set_Entry_ClearDist_Btn;
lv_obj_t * ui_Set_Entry_ClearDist_Btn_Label;
lv_obj_t * ui_Set_Entry_SetWheelLen;
lv_obj_t * ui_Set_Entry_SetWheelLen_Label;
lv_obj_t * ui_Set_Entry_SetWheelLen_Spin;
void ui_event_Set_Entry_SetWheelLen_Plus(lv_event_t * e);
lv_obj_t * ui_Set_Entry_SetWheelLen_Plus;
lv_obj_t * ui_Set_Entry_SetWheelLen_Plus_Label;
void ui_event_Set_Entry_SetWheelLen_Minus(lv_event_t * e);
lv_obj_t * ui_Set_Entry_SetWheelLen_Minus;
lv_obj_t * ui_Set_Entry_SetWheelLen_Minus_Label;
lv_obj_t * ui____initial_actions0;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Main_RoadApp(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_RoadApp, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_RoadApp_screen_init);
    }
}
void ui_event_Main_SetApp(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_SettingsApp, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_SettingsApp_screen_init);
    }
}
void ui_event_Road_Back_Btn(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_MainApp, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_MainApp_screen_init);
    }
}
void ui_event_Set_TopBackBtn(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_MainApp, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_MainApp_screen_init);
    }
}
void ui_event_Set_Entry_SetWheelLen_Plus(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_spinbox_step(ui_Set_Entry_SetWheelLen_Spin, 1);
    }
}
void ui_event_Set_Entry_SetWheelLen_Minus(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_spinbox_step(ui_Set_Entry_SetWheelLen_Spin, -1);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_MainApp_screen_init();
    ui_RoadApp_screen_init();
    ui_SettingsApp_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_MainApp);
}
