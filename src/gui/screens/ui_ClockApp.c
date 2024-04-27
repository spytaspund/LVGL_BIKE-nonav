// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: LVGL_BIKE

#include "../ui.h"

void ui_ClockApp_screen_init(void)
{
    ui_ClockApp = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ClockApp, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Clock_Label = lv_label_create(ui_ClockApp);
    lv_obj_set_width(ui_Clock_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Clock_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Clock_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Clock_Label, "18:41");
    lv_obj_set_style_text_color(ui_Clock_Label, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Clock_Label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Clock_Label, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Clock_Back_Btn = lv_btn_create(ui_ClockApp);
    lv_obj_set_width(ui_Clock_Back_Btn, 48);
    lv_obj_set_height(ui_Clock_Back_Btn, 32);
    lv_obj_set_x(ui_Clock_Back_Btn, 6);
    lv_obj_set_y(ui_Clock_Back_Btn, 6);
    lv_obj_add_flag(ui_Clock_Back_Btn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Clock_Back_Btn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Clock_Back_Btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Clock_Back_Btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Clock_Back_Btn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Clock_Back_Btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_Clock_Back_Btn, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_Clock_Back_Btn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_Clock_Back_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Clock_Back_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Clock_Back_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Clock_Back_Btn_Label = lv_label_create(ui_Clock_Back_Btn);
    lv_obj_set_width(ui_Clock_Back_Btn_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Clock_Back_Btn_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Clock_Back_Btn_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Clock_Back_Btn_Label, ">");
    lv_obj_set_style_text_color(ui_Clock_Back_Btn_Label, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Clock_Back_Btn_Label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Clock_Back_Btn_Label, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Clock_Change_Btn = lv_btn_create(ui_ClockApp);
    lv_obj_set_width(ui_Clock_Change_Btn, 32);
    lv_obj_set_height(ui_Clock_Change_Btn, 32);
    lv_obj_set_x(ui_Clock_Change_Btn, 62);
    lv_obj_set_y(ui_Clock_Change_Btn, 6);
    lv_obj_add_flag(ui_Clock_Change_Btn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Clock_Change_Btn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Clock_Change_Btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Clock_Change_Btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Clock_Change_Btn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Clock_Change_Btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_Clock_Change_Btn, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_Clock_Change_Btn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_Clock_Change_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Clock_Change_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Clock_Change_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Clock_Change_Btn_Label = lv_label_create(ui_Clock_Change_Btn);
    lv_obj_set_width(ui_Clock_Change_Btn_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Clock_Change_Btn_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Clock_Change_Btn_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Clock_Change_Btn_Label, "R");
    lv_obj_set_style_text_color(ui_Clock_Change_Btn_Label, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Clock_Change_Btn_Label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Clock_Change_Btn_Label, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Clock_DarkTheme_Btn = lv_btn_create(ui_ClockApp);
    lv_obj_set_width(ui_Clock_DarkTheme_Btn, 32);
    lv_obj_set_height(ui_Clock_DarkTheme_Btn, 32);
    lv_obj_set_x(ui_Clock_DarkTheme_Btn, 102);
    lv_obj_set_y(ui_Clock_DarkTheme_Btn, 6);
    lv_obj_add_flag(ui_Clock_DarkTheme_Btn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Clock_DarkTheme_Btn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Clock_DarkTheme_Btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Clock_DarkTheme_Btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Clock_DarkTheme_Btn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Clock_DarkTheme_Btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_Clock_DarkTheme_Btn, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_Clock_DarkTheme_Btn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_Clock_DarkTheme_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Clock_DarkTheme_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Clock_DarkTheme_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Clock_DarkTheme_Btn_Label = lv_label_create(ui_Clock_DarkTheme_Btn);
    lv_obj_set_width(ui_Clock_DarkTheme_Btn_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Clock_DarkTheme_Btn_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Clock_DarkTheme_Btn_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Clock_DarkTheme_Btn_Label, "N");
    lv_obj_set_style_text_color(ui_Clock_DarkTheme_Btn_Label, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Clock_DarkTheme_Btn_Label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Clock_DarkTheme_Btn_Label, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Clock_Back_Btn, ui_event_Clock_Back_Btn, LV_EVENT_ALL, NULL);

}
