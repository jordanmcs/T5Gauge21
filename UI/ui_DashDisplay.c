// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.3
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

lv_obj_t *ui_DashDisplay = NULL;lv_obj_t *ui_LambdaArc = NULL;lv_obj_t *ui_ThrottleArc = NULL;lv_obj_t *ui_rpmLbl = NULL;lv_obj_t *ui_groupContainer = NULL;lv_obj_t *ui_vbar = NULL;lv_obj_t *ui_boostLbl = NULL;lv_obj_t *ui_boostValLbl = NULL;lv_obj_t *ui_boostmaxValLbl = NULL;lv_obj_t *ui_aitLbl = NULL;lv_obj_t *ui_aitValLbl = NULL;lv_obj_t *ui_aitmaxValLbl = NULL;lv_obj_t *ui_maxLbl = NULL;lv_obj_t *ui_speedLbl = NULL;lv_obj_t *ui_ShiftUpLbl = NULL;
// event funtions
void ui_event_DashDisplay( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( &ui_SingleScr, LV_SCR_LOAD_ANIM_MOVE_TOP, 200, 0, &ui_SingleScr_screen_init);
}
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( &ui_SettingsScr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, &ui_SettingsScr_screen_init);
}
}

// build funtions

void ui_DashDisplay_screen_init(void)
{
ui_DashDisplay = lv_obj_create(NULL);
lv_obj_clear_flag( ui_DashDisplay, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_DashDisplay, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_DashDisplay, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LambdaArc = lv_arc_create(ui_DashDisplay);
lv_obj_set_width( ui_LambdaArc, 480);
lv_obj_set_height( ui_LambdaArc, 480);
lv_obj_set_align( ui_LambdaArc, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_LambdaArc, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_arc_set_range(ui_LambdaArc, 0,255);
lv_arc_set_value(ui_LambdaArc, 50);
lv_arc_set_bg_angles(ui_LambdaArc,180,360);
lv_obj_set_style_arc_color(ui_LambdaArc, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_LambdaArc, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_arc_rounded(ui_LambdaArc, false, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_LambdaArc, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_LambdaArc, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_outline_width(ui_LambdaArc, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_outline_pad(ui_LambdaArc, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_color(ui_LambdaArc, lv_color_hex(0x000000), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_LambdaArc, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_LambdaArc, 25, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_rounded(ui_LambdaArc, false, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_LambdaArc, 0, LV_PART_KNOB| LV_STATE_DEFAULT);
ui_object_set_themeable_style_property(ui_LambdaArc, LV_PART_KNOB| LV_STATE_DEFAULT, LV_STYLE_BG_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(ui_LambdaArc, LV_PART_KNOB| LV_STATE_DEFAULT, LV_STYLE_BG_OPA, _ui_theme_alpha_Green);

ui_ThrottleArc = lv_arc_create(ui_DashDisplay);
lv_obj_set_width( ui_ThrottleArc, 480);
lv_obj_set_height( ui_ThrottleArc, 480);
lv_obj_set_align( ui_ThrottleArc, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_ThrottleArc, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_arc_set_range(ui_ThrottleArc, 0,255);
lv_arc_set_value(ui_ThrottleArc, 10);
lv_arc_set_bg_angles(ui_ThrottleArc,30,150);
lv_arc_set_mode(ui_ThrottleArc, LV_ARC_MODE_REVERSE);
lv_obj_set_style_border_width(ui_ThrottleArc, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_arc_rounded(ui_ThrottleArc, false, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_object_set_themeable_style_property(ui_ThrottleArc, LV_PART_INDICATOR| LV_STATE_DEFAULT, LV_STYLE_ARC_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(ui_ThrottleArc, LV_PART_INDICATOR| LV_STATE_DEFAULT, LV_STYLE_ARC_OPA, _ui_theme_alpha_Green);
lv_obj_set_style_arc_width(ui_ThrottleArc, 20, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_rounded(ui_ThrottleArc, false, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_ThrottleArc, lv_color_hex(0x02B019), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ThrottleArc, 0, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_rpmLbl = lv_label_create(ui_DashDisplay);
lv_obj_set_width( ui_rpmLbl, 248);
lv_obj_set_height( ui_rpmLbl, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_rpmLbl, -10 );
lv_obj_set_y( ui_rpmLbl, -125 );
lv_obj_set_align( ui_rpmLbl, LV_ALIGN_CENTER );
lv_label_set_text(ui_rpmLbl,"8888");
ui_object_set_themeable_style_property(ui_rpmLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(ui_rpmLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_Green);
lv_obj_set_style_text_align(ui_rpmLbl, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_rpmLbl, &ui_font_SevenSegMd, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_groupContainer = lv_obj_create(ui_DashDisplay);
lv_obj_remove_style_all(ui_groupContainer);
lv_obj_set_width( ui_groupContainer, 400);
lv_obj_set_height( ui_groupContainer, 350);
lv_obj_set_x( ui_groupContainer, 0 );
lv_obj_set_y( ui_groupContainer, 15 );
lv_obj_set_align( ui_groupContainer, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_groupContainer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_vbar = lv_obj_create(ui_groupContainer);
lv_obj_remove_style_all(ui_vbar);
lv_obj_set_width( ui_vbar, 200);
lv_obj_set_height( ui_vbar, 120);
lv_obj_set_x( ui_vbar, 100 );
lv_obj_set_y( ui_vbar, 0 );
lv_obj_set_align( ui_vbar, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_vbar, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_border_color(ui_vbar, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_vbar, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_vbar, 2, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_vbar, LV_BORDER_SIDE_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_boostLbl = lv_label_create(ui_groupContainer);
lv_obj_set_width( ui_boostLbl, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_boostLbl, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_boostLbl, -100 );
lv_obj_set_y( ui_boostLbl, -50 );
lv_obj_set_align( ui_boostLbl, LV_ALIGN_CENTER );
lv_label_set_text(ui_boostLbl,"BOOST");
lv_obj_set_style_text_color(ui_boostLbl, lv_color_hex(0xECC200), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_boostLbl, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_boostLbl, &ui_font_SevenSegLbl, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_boostValLbl = lv_label_create(ui_groupContainer);
lv_obj_set_width( ui_boostValLbl, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_boostValLbl, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_boostValLbl, -110 );
lv_obj_set_y( ui_boostValLbl, 10 );
lv_obj_set_align( ui_boostValLbl, LV_ALIGN_CENTER );
lv_label_set_text(ui_boostValLbl,"--");
ui_object_set_themeable_style_property(ui_boostValLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(ui_boostValLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_Green);
lv_obj_set_style_text_font(ui_boostValLbl, &ui_font_SeveSegSm, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_boostmaxValLbl = lv_label_create(ui_groupContainer);
lv_obj_set_width( ui_boostmaxValLbl, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_boostmaxValLbl, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_boostmaxValLbl, -110 );
lv_obj_set_y( ui_boostmaxValLbl, 100 );
lv_obj_set_align( ui_boostmaxValLbl, LV_ALIGN_CENTER );
lv_label_set_text(ui_boostmaxValLbl,"0.0");
ui_object_set_themeable_style_property(ui_boostmaxValLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(ui_boostmaxValLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_Green);
lv_obj_set_style_text_font(ui_boostmaxValLbl, &ui_font_SeveSegSm, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_aitLbl = lv_label_create(ui_groupContainer);
lv_obj_set_width( ui_aitLbl, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_aitLbl, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_aitLbl, 100 );
lv_obj_set_y( ui_aitLbl, -50 );
lv_obj_set_align( ui_aitLbl, LV_ALIGN_CENTER );
lv_label_set_text(ui_aitLbl,"AIT");
lv_obj_set_style_text_color(ui_aitLbl, lv_color_hex(0xDCB502), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_aitLbl, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_aitLbl, &ui_font_SevenSegLbl, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_aitValLbl = lv_label_create(ui_groupContainer);
lv_obj_set_width( ui_aitValLbl, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_aitValLbl, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_aitValLbl, -50 );
lv_obj_set_y( ui_aitValLbl, 10 );
lv_obj_set_align( ui_aitValLbl, LV_ALIGN_RIGHT_MID );
lv_label_set_text(ui_aitValLbl,"--");
ui_object_set_themeable_style_property(ui_aitValLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(ui_aitValLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_Green);
lv_obj_set_style_text_font(ui_aitValLbl, &ui_font_SeveSegSm, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_aitmaxValLbl = lv_label_create(ui_groupContainer);
lv_obj_set_width( ui_aitmaxValLbl, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_aitmaxValLbl, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_aitmaxValLbl, -50 );
lv_obj_set_y( ui_aitmaxValLbl, 100 );
lv_obj_set_align( ui_aitmaxValLbl, LV_ALIGN_RIGHT_MID );
lv_label_set_text(ui_aitmaxValLbl,"0");
ui_object_set_themeable_style_property(ui_aitmaxValLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(ui_aitmaxValLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_Green);
lv_obj_set_style_text_font(ui_aitmaxValLbl, &ui_font_SeveSegSm, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_maxLbl = lv_label_create(ui_groupContainer);
lv_obj_set_width( ui_maxLbl, 300);
lv_obj_set_height( ui_maxLbl, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_maxLbl, 1 );
lv_obj_set_y( ui_maxLbl, 92 );
lv_obj_set_align( ui_maxLbl, LV_ALIGN_CENTER );
lv_label_set_text(ui_maxLbl,"max");
lv_obj_clear_flag( ui_maxLbl, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE );    /// Flags
lv_obj_set_style_text_color(ui_maxLbl, lv_color_hex(0xDCB502), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_maxLbl, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_maxLbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_maxLbl, &ui_font_SevenSegLbl, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_maxLbl, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_maxLbl, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_maxLbl, 2, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_maxLbl, LV_BORDER_SIDE_TOP, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_maxLbl, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_maxLbl, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_maxLbl, 20, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_maxLbl, 10, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_speedLbl = lv_label_create(ui_groupContainer);
lv_obj_set_width( ui_speedLbl, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_speedLbl, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_speedLbl, -184 );
lv_obj_set_y( ui_speedLbl, 152 );
lv_obj_set_align( ui_speedLbl, LV_ALIGN_RIGHT_MID );
lv_label_set_text(ui_speedLbl,"0");
ui_object_set_themeable_style_property(ui_speedLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(ui_speedLbl, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_Green);
lv_obj_set_style_text_font(ui_speedLbl, &ui_font_SeveSegSm, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ShiftUpLbl = lv_obj_create(ui_DashDisplay);
lv_obj_set_width( ui_ShiftUpLbl, 50);
lv_obj_set_height( ui_ShiftUpLbl, 50);
lv_obj_set_x( ui_ShiftUpLbl, 140 );
lv_obj_set_y( ui_ShiftUpLbl, -110 );
lv_obj_set_align( ui_ShiftUpLbl, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_ShiftUpLbl, LV_OBJ_FLAG_HIDDEN );   /// Flags
lv_obj_clear_flag( ui_ShiftUpLbl, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_ShiftUpLbl, 25, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_ShiftUpLbl, lv_color_hex(0xDCB502), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ShiftUpLbl, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_DashDisplay, ui_event_DashDisplay, LV_EVENT_ALL, NULL);

}

void ui_DashDisplay_screen_destroy(void)
{
   if (ui_DashDisplay) lv_obj_del(ui_DashDisplay);

// NULL screen variables
ui_DashDisplay= NULL;
ui_LambdaArc= NULL;
ui_ThrottleArc= NULL;
ui_rpmLbl= NULL;
ui_groupContainer= NULL;
ui_vbar= NULL;
ui_boostLbl= NULL;
ui_boostValLbl= NULL;
ui_boostmaxValLbl= NULL;
ui_aitLbl= NULL;
ui_aitValLbl= NULL;
ui_aitmaxValLbl= NULL;
ui_maxLbl= NULL;
ui_speedLbl= NULL;
ui_ShiftUpLbl= NULL;

}
