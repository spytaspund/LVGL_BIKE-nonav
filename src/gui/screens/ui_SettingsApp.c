// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: LVGL_BIKE

#include "../ui.h"

void ui_SettingsApp_screen_init(void)
{
    ui_SettingsApp = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_SettingsApp, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Set_TopBar = lv_obj_create(ui_SettingsApp);
    lv_obj_set_width(ui_Set_TopBar, 320);
    lv_obj_set_height(ui_Set_TopBar, 48);
    lv_obj_set_x(ui_Set_TopBar, 0);
    lv_obj_set_y(ui_Set_TopBar, -8);
    lv_obj_clear_flag(ui_Set_TopBar, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Set_TopBar, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_TopLabel = lv_label_create(ui_Set_TopBar);
    lv_obj_set_width(ui_Set_TopLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Set_TopLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Set_TopLabel, 38);
    lv_obj_set_y(ui_Set_TopLabel, -1);
    lv_label_set_text(ui_Set_TopLabel, "settings");
    lv_obj_set_style_text_letter_space(ui_Set_TopLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Set_TopLabel, -6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Set_TopLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Set_TopLabel, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_TopIcon = lv_label_create(ui_Set_TopBar);
    lv_obj_set_width(ui_Set_TopIcon, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Set_TopIcon, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Set_TopIcon, 4);
    lv_obj_set_y(ui_Set_TopIcon, 2);
    lv_obj_set_align(ui_Set_TopIcon, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_Set_TopIcon, "S");
    lv_obj_set_style_text_font(ui_Set_TopIcon, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_TopBackBtn = lv_btn_create(ui_Set_TopBar);
    lv_obj_set_width(ui_Set_TopBackBtn, 48);
    lv_obj_set_height(ui_Set_TopBackBtn, 32);
    lv_obj_set_x(ui_Set_TopBackBtn, 10);
    lv_obj_set_y(ui_Set_TopBackBtn, 2);
    lv_obj_set_align(ui_Set_TopBackBtn, LV_ALIGN_RIGHT_MID);
    lv_obj_add_flag(ui_Set_TopBackBtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Set_TopBackBtn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Set_TopBackBtn, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Set_TopBackBtn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Set_TopBackBtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Set_TopBackBtn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Set_TopBackBtn, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Set_TopBackBtn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Set_TopBackBtn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Set_TopBackBtn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Set_TopBackBtn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Set_TopBackBtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Set_TopBackBtn, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_TopBackBtn_Label = lv_label_create(ui_Set_TopBackBtn);
    lv_obj_set_width(ui_Set_TopBackBtn_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Set_TopBackBtn_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Set_TopBackBtn_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Set_TopBackBtn_Label, ">");

    ui_Set_Main = lv_obj_create(ui_SettingsApp);
    lv_obj_set_width(ui_Set_Main, 320);
    lv_obj_set_height(ui_Set_Main, 208);
    lv_obj_set_x(ui_Set_Main, 0);
    lv_obj_set_y(ui_Set_Main, 40);
    lv_obj_set_flex_flow(ui_Set_Main, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_Set_Main, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_scroll_dir(ui_Set_Main, LV_DIR_VER);
    lv_obj_set_style_border_width(ui_Set_Main, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_ClearDist = lv_obj_create(ui_Set_Main);
    lv_obj_set_width(ui_Set_Entry_ClearDist, 295);
    lv_obj_set_height(ui_Set_Entry_ClearDist, 48);
    lv_obj_set_x(ui_Set_Entry_ClearDist, 1);
    lv_obj_set_y(ui_Set_Entry_ClearDist, -1);
    lv_obj_set_align(ui_Set_Entry_ClearDist, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Set_Entry_ClearDist, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Set_Entry_ClearDist, 15, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_ClearDist_Label = lv_label_create(ui_Set_Entry_ClearDist);
    lv_obj_set_width(ui_Set_Entry_ClearDist_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Set_Entry_ClearDist_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Set_Entry_ClearDist_Label, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_Set_Entry_ClearDist_Label, "Clear Distance");
    lv_obj_set_style_text_align(ui_Set_Entry_ClearDist_Label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Set_Entry_ClearDist_Label, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_ClearDist_Btn = lv_btn_create(ui_Set_Entry_ClearDist);
    lv_obj_set_width(ui_Set_Entry_ClearDist_Btn, 72);
    lv_obj_set_height(ui_Set_Entry_ClearDist_Btn, 32);
    lv_obj_set_x(ui_Set_Entry_ClearDist_Btn, 7);
    lv_obj_set_y(ui_Set_Entry_ClearDist_Btn, 0);
    lv_obj_set_align(ui_Set_Entry_ClearDist_Btn, LV_ALIGN_RIGHT_MID);
    lv_obj_add_flag(ui_Set_Entry_ClearDist_Btn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Set_Entry_ClearDist_Btn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Set_Entry_ClearDist_Btn, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Set_Entry_ClearDist_Btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Set_Entry_ClearDist_Btn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Set_Entry_ClearDist_Btn, lv_color_hex(0xE72929), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_Set_Entry_ClearDist_Btn, 150, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_Set_Entry_ClearDist_Btn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_Set_Entry_ClearDist_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Set_Entry_ClearDist_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Set_Entry_ClearDist_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Set_Entry_ClearDist_Btn, lv_color_hex(0xE72929), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Set_Entry_ClearDist_Btn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_Set_Entry_ClearDist_Btn, 7, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(ui_Set_Entry_ClearDist_Btn, lv_color_hex(0xE72929), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Set_Entry_ClearDist_Btn, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_text_color(ui_Set_Entry_ClearDist_Btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_text_opa(ui_Set_Entry_ClearDist_Btn, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Set_Entry_ClearDist_Btn_Label = lv_label_create(ui_Set_Entry_ClearDist_Btn);
    lv_obj_set_width(ui_Set_Entry_ClearDist_Btn_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Set_Entry_ClearDist_Btn_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Set_Entry_ClearDist_Btn_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Set_Entry_ClearDist_Btn_Label, "Clear");

    ui_Set_Entry_SetWheelLen = lv_obj_create(ui_Set_Main);
    lv_obj_set_width(ui_Set_Entry_SetWheelLen, 295);
    lv_obj_set_height(ui_Set_Entry_SetWheelLen, 48);
    lv_obj_set_x(ui_Set_Entry_SetWheelLen, 1);
    lv_obj_set_y(ui_Set_Entry_SetWheelLen, -1);
    lv_obj_set_align(ui_Set_Entry_SetWheelLen, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Set_Entry_SetWheelLen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Set_Entry_SetWheelLen, 15, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_SetWheelLen_Label = lv_label_create(ui_Set_Entry_SetWheelLen);
    lv_obj_set_width(ui_Set_Entry_SetWheelLen_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Set_Entry_SetWheelLen_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Set_Entry_SetWheelLen_Label, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_Set_Entry_SetWheelLen_Label, "Wheel Length");
    lv_obj_set_style_text_align(ui_Set_Entry_SetWheelLen_Label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Set_Entry_SetWheelLen_Label, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_SetWheelLen_Spin = lv_spinbox_create(ui_Set_Entry_SetWheelLen);
    lv_obj_set_width(ui_Set_Entry_SetWheelLen_Spin, 64);
    lv_obj_set_height(ui_Set_Entry_SetWheelLen_Spin, 32);
    lv_obj_set_x(ui_Set_Entry_SetWheelLen_Spin, -30);
    lv_obj_set_y(ui_Set_Entry_SetWheelLen_Spin, 0);
    lv_obj_set_align(ui_Set_Entry_SetWheelLen_Spin, LV_ALIGN_RIGHT_MID);
    lv_spinbox_set_digit_format(ui_Set_Entry_SetWheelLen_Spin, 4, 1);
    lv_spinbox_set_range(ui_Set_Entry_SetWheelLen_Spin, 0, 5000);
    lv_spinbox_set_cursor_pos(ui_Set_Entry_SetWheelLen_Spin, 1 - 1);
    lv_spinbox_set_value(ui_Set_Entry_SetWheelLen_Spin, 2007);
    lv_obj_set_style_radius(ui_Set_Entry_SetWheelLen_Spin, 7, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(ui_Set_Entry_SetWheelLen_Spin, 2, LV_PART_CURSOR | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Set_Entry_SetWheelLen_Spin, LV_TEXT_ALIGN_CENTER, LV_PART_CURSOR | LV_STATE_DEFAULT);

    ui_Set_Entry_SetWheelLen_Plus = lv_btn_create(ui_Set_Entry_SetWheelLen);
    lv_obj_set_width(ui_Set_Entry_SetWheelLen_Plus, 32);
    lv_obj_set_height(ui_Set_Entry_SetWheelLen_Plus, 32);
    lv_obj_set_x(ui_Set_Entry_SetWheelLen_Plus, 7);
    lv_obj_set_y(ui_Set_Entry_SetWheelLen_Plus, 0);
    lv_obj_set_align(ui_Set_Entry_SetWheelLen_Plus, LV_ALIGN_RIGHT_MID);
    lv_obj_add_flag(ui_Set_Entry_SetWheelLen_Plus, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Set_Entry_SetWheelLen_Plus, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Set_Entry_SetWheelLen_Plus, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Set_Entry_SetWheelLen_Plus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Set_Entry_SetWheelLen_Plus, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_Set_Entry_SetWheelLen_Plus, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_Set_Entry_SetWheelLen_Plus, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_Set_Entry_SetWheelLen_Plus, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Set_Entry_SetWheelLen_Plus, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Set_Entry_SetWheelLen_Plus, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_SetWheelLen_Plus_Label = lv_label_create(ui_Set_Entry_SetWheelLen_Plus);
    lv_obj_set_width(ui_Set_Entry_SetWheelLen_Plus_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Set_Entry_SetWheelLen_Plus_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Set_Entry_SetWheelLen_Plus_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Set_Entry_SetWheelLen_Plus_Label, "+");
    lv_obj_set_style_text_color(ui_Set_Entry_SetWheelLen_Plus_Label, lv_color_hex(0x000000),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Set_Entry_SetWheelLen_Plus_Label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Set_Entry_SetWheelLen_Plus_Label, &lv_font_montserrat_36,
                               LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_SetWheelLen_Minus = lv_btn_create(ui_Set_Entry_SetWheelLen);
    lv_obj_set_width(ui_Set_Entry_SetWheelLen_Minus, 32);
    lv_obj_set_height(ui_Set_Entry_SetWheelLen_Minus, 32);
    lv_obj_set_x(ui_Set_Entry_SetWheelLen_Minus, -100);
    lv_obj_set_y(ui_Set_Entry_SetWheelLen_Minus, 0);
    lv_obj_set_align(ui_Set_Entry_SetWheelLen_Minus, LV_ALIGN_RIGHT_MID);
    lv_obj_add_flag(ui_Set_Entry_SetWheelLen_Minus, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Set_Entry_SetWheelLen_Minus, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Set_Entry_SetWheelLen_Minus, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Set_Entry_SetWheelLen_Minus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Set_Entry_SetWheelLen_Minus, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_Set_Entry_SetWheelLen_Minus, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_Set_Entry_SetWheelLen_Minus, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_Set_Entry_SetWheelLen_Minus, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Set_Entry_SetWheelLen_Minus, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Set_Entry_SetWheelLen_Minus, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Set_Entry_SetWheelLen_Minus, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Set_Entry_SetWheelLen_Minus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_SetWheelLen_Minus_Label = lv_label_create(ui_Set_Entry_SetWheelLen_Minus);
    lv_obj_set_width(ui_Set_Entry_SetWheelLen_Minus_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Set_Entry_SetWheelLen_Minus_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Set_Entry_SetWheelLen_Minus_Label, 0);
    lv_obj_set_y(ui_Set_Entry_SetWheelLen_Minus_Label, -3);
    lv_obj_set_align(ui_Set_Entry_SetWheelLen_Minus_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Set_Entry_SetWheelLen_Minus_Label, "-");
    lv_obj_set_style_text_font(ui_Set_Entry_SetWheelLen_Minus_Label, &lv_font_montserrat_36,
                               LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_Debug = lv_obj_create(ui_Set_Main);
    lv_obj_set_width(ui_Set_Entry_Debug, 295);
    lv_obj_set_height(ui_Set_Entry_Debug, 48);
    lv_obj_set_x(ui_Set_Entry_Debug, 1);
    lv_obj_set_y(ui_Set_Entry_Debug, -1);
    lv_obj_set_align(ui_Set_Entry_Debug, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Set_Entry_Debug, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Set_Entry_Debug, 15, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_Debug_Label = lv_label_create(ui_Set_Entry_Debug);
    lv_obj_set_width(ui_Set_Entry_Debug_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Set_Entry_Debug_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Set_Entry_Debug_Label, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_Set_Entry_Debug_Label, "Debug mode");
    lv_obj_set_style_text_align(ui_Set_Entry_Debug_Label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Set_Entry_Debug_Label, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Set_Entry_Debug_Swt = lv_switch_create(ui_Set_Entry_Debug);
    lv_obj_set_width(ui_Set_Entry_Debug_Swt, 48);
    lv_obj_set_height(ui_Set_Entry_Debug_Swt, 24);
    lv_obj_set_x(ui_Set_Entry_Debug_Swt, 4);
    lv_obj_set_y(ui_Set_Entry_Debug_Swt, 0);
    lv_obj_set_align(ui_Set_Entry_Debug_Swt, LV_ALIGN_RIGHT_MID);


    lv_obj_add_event_cb(ui_Set_TopBackBtn, ui_event_Set_TopBackBtn, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Set_Entry_SetWheelLen_Plus, ui_event_Set_Entry_SetWheelLen_Plus, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Set_Entry_SetWheelLen_Minus, ui_event_Set_Entry_SetWheelLen_Minus, LV_EVENT_ALL, NULL);

}
