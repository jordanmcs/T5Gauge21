// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.3
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_event_comp_Gauge_unit( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
lv_obj_t **comp_Gauge = lv_event_get_user_data(e);

if ( event_code == LV_EVENT_CLICKED) {
      ToggleImperial( e );
}
}

// COMPONENT Gauge

lv_obj_t *ui_Gauge_create(lv_obj_t *comp_parent) {

lv_obj_t *cui_Gauge;
cui_Gauge = lv_obj_create(comp_parent);
lv_obj_remove_style_all(cui_Gauge);
lv_obj_set_width( cui_Gauge, 480);
lv_obj_set_height( cui_Gauge, 480);
lv_obj_set_x( cui_Gauge, 124 );
lv_obj_set_y( cui_Gauge, 244 );
lv_obj_set_align( cui_Gauge, LV_ALIGN_CENTER );
lv_obj_clear_flag( cui_Gauge, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags

lv_obj_t *cui_name;
cui_name = lv_label_create(cui_Gauge);
lv_obj_set_width( cui_name, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( cui_name, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( cui_name, 0 );
lv_obj_set_y( cui_name, -140 );
lv_obj_set_align( cui_name, LV_ALIGN_CENTER );
lv_label_set_text(cui_name,"batt");
lv_obj_set_style_text_color(cui_name, lv_color_hex(0xDEB600), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(cui_name, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(cui_name, &ui_font_SeveSegSm, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_t *cui_value;
cui_value = lv_label_create(cui_Gauge);
lv_obj_set_width( cui_value, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( cui_value, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( cui_value, LV_ALIGN_CENTER );
lv_label_set_text(cui_value,"12.7");
lv_obj_clear_flag( cui_value, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scroll_snap_x(cui_value, LV_SCROLL_SNAP_START);
ui_object_set_themeable_style_property(cui_value, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(cui_value, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_Green);
lv_obj_set_style_text_font(cui_value, &ui_font_SeveSegLg, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_t *cui_unit;
cui_unit = lv_label_create(cui_Gauge);
lv_obj_set_width( cui_unit, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( cui_unit, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( cui_unit, 164 );
lv_obj_set_y( cui_unit, -23 );
lv_obj_set_align( cui_unit, LV_ALIGN_CENTER );
lv_label_set_text(cui_unit,"V");
lv_obj_add_flag( cui_unit, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( cui_unit, LV_OBJ_FLAG_PRESS_LOCK );    /// Flags
lv_obj_set_style_text_color(cui_unit, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(cui_unit, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(cui_unit, &ui_font_SeveSegSm, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(cui_unit, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(cui_unit, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_color(cui_unit, lv_color_hex(0xFAFAFA), LV_PART_MAIN | LV_STATE_PRESSED );
lv_obj_set_style_text_opa(cui_unit, 255, LV_PART_MAIN| LV_STATE_PRESSED);

lv_obj_t *cui_maxValue;
cui_maxValue = lv_label_create(cui_Gauge);
lv_obj_set_width( cui_maxValue, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( cui_maxValue, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( cui_maxValue, 0 );
lv_obj_set_y( cui_maxValue, 135 );
lv_obj_set_align( cui_maxValue, LV_ALIGN_CENTER );
lv_label_set_text(cui_maxValue,"--");
ui_object_set_themeable_style_property(cui_maxValue, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
ui_object_set_themeable_style_property(cui_maxValue, LV_PART_MAIN| LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_Green);
lv_obj_set_style_text_font(cui_maxValue, &ui_font_SeveSegSm, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_t ** children = lv_mem_alloc(sizeof(lv_obj_t *) * _UI_COMP_GAUGE_NUM);
children[UI_COMP_GAUGE_GAUGE] = cui_Gauge;
children[UI_COMP_GAUGE_NAME] = cui_name;
children[UI_COMP_GAUGE_VALUE] = cui_value;
children[UI_COMP_GAUGE_UNIT] = cui_unit;
children[UI_COMP_GAUGE_MAXVALUE] = cui_maxValue;
lv_obj_add_event_cb(cui_Gauge, get_component_child_event_cb, LV_EVENT_GET_COMP_CHILD, children);
lv_obj_add_event_cb(cui_Gauge, del_component_child_event_cb, LV_EVENT_DELETE, children);
lv_obj_add_event_cb(cui_unit, ui_event_comp_Gauge_unit, LV_EVENT_ALL, children);
ui_comp_Gauge_create_hook(cui_Gauge);
return cui_Gauge; 
}

