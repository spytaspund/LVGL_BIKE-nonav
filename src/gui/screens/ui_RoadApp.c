// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: LVGL_BIKE

#include "../ui.h"

void ui_RoadApp_screen_init(void)
{
    ui_RoadApp = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_RoadApp, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Road_Speed = lv_obj_create(ui_RoadApp);
    lv_obj_set_width(ui_Road_Speed, 150);
    lv_obj_set_height(ui_Road_Speed, 150);
    lv_obj_set_x(ui_Road_Speed, 16);
    lv_obj_set_y(ui_Road_Speed, 16);
    lv_obj_clear_flag(ui_Road_Speed, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE |
                      LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                      LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_style_radius(ui_Road_Speed, 20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Road_Speed_Arc = lv_arc_create(ui_Road_Speed);
    lv_obj_set_width(ui_Road_Speed_Arc, 120);
    lv_obj_set_height(ui_Road_Speed_Arc, 120);
    lv_obj_set_x(ui_Road_Speed_Arc, 0);
    lv_obj_set_y(ui_Road_Speed_Arc, 5);
    lv_obj_set_align(ui_Road_Speed_Arc, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Road_Speed_Arc,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                      LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_arc_set_range(ui_Road_Speed_Arc, 0, 40);
    lv_arc_set_value(ui_Road_Speed_Arc, 2);
    lv_obj_set_style_arc_color(ui_Road_Speed_Arc, lv_color_hex(0x2D2D2D), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_Road_Speed_Arc, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_Road_Speed_Arc, 12, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(ui_Road_Speed_Arc, lv_color_hex(0xD20000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_Road_Speed_Arc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_Road_Speed_Arc, 12, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Road_Speed_Arc, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Road_Speed_Arc, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_Road_Speed_Label = lv_label_create(ui_Road_Speed_Arc);
    lv_obj_set_width(ui_Road_Speed_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Road_Speed_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Road_Speed_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Road_Speed_Label, "2");
    lv_obj_clear_flag(ui_Road_Speed_Label,
                      LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE |
                      LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                      LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_style_text_color(ui_Road_Speed_Label, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Road_Speed_Label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Road_Speed_Label, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Road_Kmh_Label = lv_label_create(ui_Road_Speed_Arc);
    lv_obj_set_width(ui_Road_Kmh_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Road_Kmh_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Road_Kmh_Label, 0);
    lv_obj_set_y(ui_Road_Kmh_Label, 48);
    lv_obj_set_align(ui_Road_Kmh_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Road_Kmh_Label, "km/h");

    ui_Road_Dist = lv_obj_create(ui_RoadApp);
    lv_obj_set_width(ui_Road_Dist, 128);
    lv_obj_set_height(ui_Road_Dist, 80);
    lv_obj_set_x(ui_Road_Dist, 176);
    lv_obj_set_y(ui_Road_Dist, 16);
    lv_obj_clear_flag(ui_Road_Dist, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Road_Dist, 20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Road_Dist_Label = lv_label_create(ui_Road_Dist);
    lv_obj_set_width(ui_Road_Dist_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Road_Dist_Label, LV_SIZE_CONTENT);    /// 1
    lv_label_set_text(ui_Road_Dist_Label, "Dist");

    ui_Road_Dist_Value = lv_label_create(ui_Road_Dist);
    lv_obj_set_width(ui_Road_Dist_Value, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Road_Dist_Value, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Road_Dist_Value, 0);
    lv_obj_set_y(ui_Road_Dist_Value, 20);
    lv_label_set_text(ui_Road_Dist_Value, "3 km");
    lv_obj_set_style_text_font(ui_Road_Dist_Value, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Road_Back_Btn = lv_btn_create(ui_RoadApp);
    lv_obj_set_width(ui_Road_Back_Btn, 150);
    lv_obj_set_height(ui_Road_Back_Btn, 50);
    lv_obj_set_x(ui_Road_Back_Btn, 16);
    lv_obj_set_y(ui_Road_Back_Btn, 176);
    lv_obj_add_flag(ui_Road_Back_Btn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Road_Back_Btn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Road_Back_Btn, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Road_Back_Btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Road_Back_Btn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Road_Back_Btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_Road_Back_Btn, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_Road_Back_Btn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_Road_Back_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Road_Back_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Road_Back_Btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Road_Back_Label = lv_label_create(ui_Road_Back_Btn);
    lv_obj_set_width(ui_Road_Back_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Road_Back_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Road_Back_Label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Road_Back_Label, "<");
    lv_obj_set_style_text_color(ui_Road_Back_Label, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Road_Back_Label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Road_Back_Label, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Road_Clock = lv_obj_create(ui_RoadApp);
    lv_obj_set_width(ui_Road_Clock, 128);
    lv_obj_set_height(ui_Road_Clock, 120);
    lv_obj_set_x(ui_Road_Clock, 176);
    lv_obj_set_y(ui_Road_Clock, 104);
    lv_obj_clear_flag(ui_Road_Clock, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Road_Clock, 20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Road_Clock_Hours = lv_label_create(ui_Road_Clock);
    lv_obj_set_width(ui_Road_Clock_Hours, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Road_Clock_Hours, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Road_Clock_Hours, 8);
    lv_obj_set_y(ui_Road_Clock_Hours, 8);
    lv_label_set_text(ui_Road_Clock_Hours, "18");
    lv_obj_set_style_text_color(ui_Road_Clock_Hours, lv_color_hex(0x787878), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Road_Clock_Hours, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Road_Clock_Hours, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Road_Clock_Minutes = lv_label_create(ui_Road_Clock);
    lv_obj_set_width(ui_Road_Clock_Minutes, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Road_Clock_Minutes, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Road_Clock_Minutes, -8);
    lv_obj_set_y(ui_Road_Clock_Minutes, -8);
    lv_obj_set_align(ui_Road_Clock_Minutes, LV_ALIGN_BOTTOM_RIGHT);
    lv_label_set_text(ui_Road_Clock_Minutes, "56");
    lv_obj_set_style_text_font(ui_Road_Clock_Minutes, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Road_Back_Btn, ui_event_Road_Back_Btn, LV_EVENT_ALL, NULL);

}