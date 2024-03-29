// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: LVGL_BIKE

#include "../ui.h"

void ui_Screen1_screen_init(void)
{
ui_Screen1 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Battery_bg = lv_obj_create(ui_Screen1);
lv_obj_set_width( ui_Battery_bg, 110);
lv_obj_set_height( ui_Battery_bg, 100);
lv_obj_set_x( ui_Battery_bg, 65 );
lv_obj_set_y( ui_Battery_bg, 15 );
lv_obj_clear_flag( ui_Battery_bg, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_Battery_bg, 20, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Battery_bg, lv_color_hex(0x1B1B1B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Battery_bg, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Battery_Level = lv_label_create(ui_Battery_bg);
lv_obj_set_width( ui_Battery_Level, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Battery_Level, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Battery_Level, 0 );
lv_obj_set_y( ui_Battery_Level, 2 );
lv_obj_set_align( ui_Battery_Level, LV_ALIGN_TOP_RIGHT );
lv_label_set_text(ui_Battery_Level,"100");
lv_label_set_recolor(ui_Battery_Level,"true");
lv_obj_clear_flag( ui_Battery_Level, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_Battery_Level, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Battery_Level, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_Battery_Level, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Battery_Level, &lv_font_montserrat_22, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Charge_Icon = lv_label_create(ui_Battery_bg);
lv_obj_set_width( ui_Charge_Icon, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Charge_Icon, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Charge_Icon, 50 );
lv_obj_set_y( ui_Charge_Icon, 35 );
lv_label_set_text(ui_Charge_Icon, LV_SYMBOL_CHARGE);
lv_obj_clear_flag( ui_Charge_Icon, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_Charge_Icon, lv_color_hex(0xFFD400), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Charge_Icon, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Charge_Icon, &lv_font_montserrat_32, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Charge_Progress = lv_bar_create(ui_Battery_bg);
lv_bar_set_value(ui_Charge_Progress,10,LV_ANIM_OFF);
lv_obj_set_width( ui_Charge_Progress, 30);
lv_obj_set_height( ui_Charge_Progress, 75);
lv_obj_set_x( ui_Charge_Progress, 2 );
lv_obj_set_y( ui_Charge_Progress, -3 );
lv_obj_clear_flag( ui_Charge_Progress, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Charge_Progress, lv_color_hex(0x08FF00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Charge_Progress, 40, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_Charge_Progress, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Charge_Progress, lv_color_hex(0x08FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Charge_Progress, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);

ui_Light2 = lv_obj_create(ui_Screen1);
lv_obj_set_width( ui_Light2, 50);
lv_obj_set_height( ui_Light2, 170);
lv_obj_set_x( ui_Light2, -20 );
lv_obj_set_y( ui_Light2, 20 );
lv_obj_set_align( ui_Light2, LV_ALIGN_RIGHT_MID );
lv_obj_clear_flag( ui_Light2, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_Light2, 20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Light_Slider_2 = lv_slider_create(ui_Light2);
lv_slider_set_value( ui_Light_Slider_2, 1, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_Light_Slider_2)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_Light_Slider_2, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_Light_Slider_2, 25);
lv_obj_set_height( ui_Light_Slider_2, 120);
lv_obj_set_align( ui_Light_Slider_2, LV_ALIGN_CENTER );
lv_obj_set_style_bg_color(ui_Light_Slider_2, lv_color_hex(0x1B1B1B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Light_Slider_2, 100, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_Light_Slider_2, lv_color_hex(0xFFD400), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Light_Slider_2, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_Light_Slider_2, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Light_Slider_2, 255, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_outline_color(ui_Light_Slider_2, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_outline_opa(ui_Light_Slider_2, 125, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_outline_width(ui_Light_Slider_2, 2, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_outline_pad(ui_Light_Slider_2, 0, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Light_Slider_2, 2, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Light_Slider_2, 2, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Light_Slider_2, 2, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Light_Slider_2, 2, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_Light1 = lv_obj_create(ui_Screen1);
lv_obj_set_width( ui_Light1, 50);
lv_obj_set_height( ui_Light1, 170);
lv_obj_set_x( ui_Light1, -82 );
lv_obj_set_y( ui_Light1, 20 );
lv_obj_set_align( ui_Light1, LV_ALIGN_RIGHT_MID );
lv_obj_clear_flag( ui_Light1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_Light1, 20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Light_Slider_1 = lv_slider_create(ui_Light1);
lv_slider_set_value( ui_Light_Slider_1, 1, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_Light_Slider_1)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_Light_Slider_1, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_Light_Slider_1, 25);
lv_obj_set_height( ui_Light_Slider_1, 120);
lv_obj_set_align( ui_Light_Slider_1, LV_ALIGN_CENTER );
lv_obj_set_style_bg_color(ui_Light_Slider_1, lv_color_hex(0x1B1B1B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Light_Slider_1, 100, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_Light_Slider_1, lv_color_hex(0xFFD400), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Light_Slider_1, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_Light_Slider_1, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Light_Slider_1, 255, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_outline_color(ui_Light_Slider_1, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_outline_opa(ui_Light_Slider_1, 125, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_outline_width(ui_Light_Slider_1, 2, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_outline_pad(ui_Light_Slider_1, 0, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Light_Slider_1, 2, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Light_Slider_1, 2, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Light_Slider_1, 2, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Light_Slider_1, 2, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_Light_Switch_1 = lv_switch_create(ui_Screen1);
lv_obj_set_width( ui_Light_Switch_1, 40);
lv_obj_set_height( ui_Light_Switch_1, 25);
lv_obj_set_x( ui_Light_Switch_1, 51 );
lv_obj_set_y( ui_Light_Switch_1, 20 );
lv_obj_set_align( ui_Light_Switch_1, LV_ALIGN_TOP_MID );

lv_obj_set_style_bg_color(ui_Light_Switch_1, lv_color_hex(0xFFD400), LV_PART_INDICATOR | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_Light_Switch_1, 255, LV_PART_INDICATOR| LV_STATE_CHECKED);

ui_Light_Switch_2 = lv_switch_create(ui_Screen1);
lv_obj_set_width( ui_Light_Switch_2, 40);
lv_obj_set_height( ui_Light_Switch_2, 25);
lv_obj_set_x( ui_Light_Switch_2, 113 );
lv_obj_set_y( ui_Light_Switch_2, 20 );
lv_obj_set_align( ui_Light_Switch_2, LV_ALIGN_TOP_MID );

lv_obj_set_style_bg_color(ui_Light_Switch_2, lv_color_hex(0xFFD400), LV_PART_INDICATOR | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_Light_Switch_2, 255, LV_PART_INDICATOR| LV_STATE_CHECKED);

ui_Speedometer = lv_obj_create(ui_Screen1);
lv_obj_set_width( ui_Speedometer, 110);
lv_obj_set_height( ui_Speedometer, 100);
lv_obj_set_x( ui_Speedometer, 65 );
lv_obj_set_y( ui_Speedometer, -15 );
lv_obj_set_align( ui_Speedometer, LV_ALIGN_BOTTOM_LEFT );
lv_obj_clear_flag( ui_Speedometer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_Speedometer, 20, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Speedometer, lv_color_hex(0x1B1B1B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Speedometer, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Speedometer_Arc = lv_arc_create(ui_Speedometer);
lv_obj_set_width( ui_Speedometer_Arc, 80);
lv_obj_set_height( ui_Speedometer_Arc, 80);
lv_obj_set_x( ui_Speedometer_Arc, 0 );
lv_obj_set_y( ui_Speedometer_Arc, 5 );
lv_obj_set_align( ui_Speedometer_Arc, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_Speedometer_Arc, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_arc_set_range(ui_Speedometer_Arc, 0,40);
lv_arc_set_value(ui_Speedometer_Arc, 2);
lv_obj_set_style_arc_color(ui_Speedometer_Arc, lv_color_hex(0x2D2D2D), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_Speedometer_Arc, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_Speedometer_Arc, 8, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_arc_color(ui_Speedometer_Arc, lv_color_hex(0xD20000), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_Speedometer_Arc, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_Speedometer_Arc, 8, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_Speedometer_Arc, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Speedometer_Arc, 0, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_Speedometer_Label = lv_label_create(ui_Speedometer_Arc);
lv_obj_set_width( ui_Speedometer_Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Speedometer_Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Speedometer_Label, LV_ALIGN_CENTER );
lv_label_set_text(ui_Speedometer_Label,"2");
lv_obj_clear_flag( ui_Speedometer_Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_Speedometer_Label, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Speedometer_Label, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Speedometer_Label, &lv_font_montserrat_34, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Back_Tab = lv_btn_create(ui_Screen1);
lv_obj_set_width( ui_Back_Tab, 40);
lv_obj_set_height( ui_Back_Tab, 208);
lv_obj_set_x( ui_Back_Tab, 15 );
lv_obj_set_y( ui_Back_Tab, 0 );
lv_obj_set_align( ui_Back_Tab, LV_ALIGN_LEFT_MID );
lv_obj_clear_flag( ui_Back_Tab, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_bg_color(ui_Back_Tab, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Back_Tab, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_color(ui_Back_Tab, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_outline_opa(ui_Back_Tab, 34, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_width(ui_Back_Tab, 2, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_pad(ui_Back_Tab, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Back_Icon = lv_label_create(ui_Back_Tab);
lv_obj_set_width( ui_Back_Icon, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Back_Icon, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Back_Icon, LV_ALIGN_CENTER );
lv_label_set_text(ui_Back_Icon, LV_SYMBOL_LEFT);
lv_obj_clear_flag( ui_Back_Icon, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_Back_Icon, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Back_Icon, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Back_Icon, &lv_font_montserrat_34, LV_PART_MAIN| LV_STATE_DEFAULT);

}
