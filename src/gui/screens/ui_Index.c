// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: Function_Generator

#include "../ui.h"

void ui_Index_screen_init(void)
{
ui_Index = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Index, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_flex_flow(ui_Index,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_Index, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_set_style_bg_color(ui_Index, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Index, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Index, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Index, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Index, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Index, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Index, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Index, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel10 = lv_obj_create(ui_Index);
lv_obj_set_width( ui_Panel10, lv_pct(100));
lv_obj_set_flex_grow( ui_Panel10, 1);
lv_obj_set_align( ui_Panel10, LV_ALIGN_TOP_MID );
lv_obj_add_flag( ui_Panel10, LV_OBJ_FLAG_SCROLL_ON_FOCUS | LV_OBJ_FLAG_SCROLL_WITH_ARROW | LV_OBJ_FLAG_SCROLL_ONE );   /// Flags
lv_obj_clear_flag( ui_Panel10, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE );    /// Flags
lv_obj_set_style_radius(ui_Panel10, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Panel10, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel10, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_Panel10, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel10, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel10, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel10, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel10, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel10, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel10, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel10, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel9 = lv_obj_create(ui_Panel10);
lv_obj_set_width( ui_Panel9, 172);
lv_obj_set_height( ui_Panel9, LV_SIZE_CONTENT);   /// 320
lv_obj_set_flex_flow(ui_Panel9,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_Panel9, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel9, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Panel9, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Panel9, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel9, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_Panel9, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel9, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel9, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel9, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel9, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel9, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel9, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel9, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_wave_form_img = lv_img_create(ui_Panel9);
lv_img_set_src(ui_wave_form_img, &ui_img_sine_png);
lv_obj_set_width( ui_wave_form_img, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_wave_form_img, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_wave_form_img, 10 );
lv_obj_set_y( ui_wave_form_img, 10 );
lv_obj_add_flag( ui_wave_form_img, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_wave_form_img, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_border_color(ui_wave_form_img, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_wave_form_img, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_wave_form_img, 1, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label14 = lv_label_create(ui_Panel9);
lv_obj_set_width( ui_Label14, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label14, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label14, 0 );
lv_obj_set_y( ui_Label14, 1 );
lv_obj_set_align( ui_Label14, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label14,"OUTPUT:");
lv_obj_set_style_text_color(ui_Label14, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label14, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Label14, lv_color_hex(0x0069FF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Label14, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_Label14, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Label14, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_output_log_label = lv_label_create(ui_Panel9);
lv_obj_set_width( ui_output_log_label, lv_pct(100));
lv_obj_set_height( ui_output_log_label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_output_log_label, LV_ALIGN_CENTER );
lv_label_set_long_mode(ui_output_log_label,LV_LABEL_LONG_SCROLL);
lv_label_set_text(ui_output_log_label,"Waveform: Sine\nFrequency: 10 kHz\nVp: 5V\nVoffset: 0V\nVp-p: 5V\n");
lv_obj_set_style_text_color(ui_output_log_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_output_log_label, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel1 = lv_obj_create(ui_Panel10);
lv_obj_set_width( ui_Panel1, lv_pct(63));
lv_obj_set_height( ui_Panel1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Panel1, LV_ALIGN_TOP_RIGHT );
lv_obj_set_flex_flow(ui_Panel1,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_Panel1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Panel1, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel1, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel1, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel1, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel1, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel1, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel1, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel1, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel1, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label4 = lv_label_create(ui_Panel1);
lv_obj_set_width( ui_Label4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label4, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label4,"Waveform");
lv_obj_set_style_text_color(ui_Label4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label4, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel2 = lv_obj_create(ui_Panel1);
lv_obj_set_width( ui_Panel2, lv_pct(100));
lv_obj_set_height( ui_Panel2, LV_SIZE_CONTENT);   /// 50
lv_obj_set_align( ui_Panel2, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel2,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_Panel2, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Panel2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel2, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel2, 5, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_sine_wave_btn = lv_btn_create(ui_Panel2);
lv_obj_set_height( ui_sine_wave_btn, 45);
lv_obj_set_width( ui_sine_wave_btn, lv_pct(30));
lv_obj_set_align( ui_sine_wave_btn, LV_ALIGN_CENTER );
lv_obj_add_state( ui_sine_wave_btn, LV_STATE_CHECKED );     /// States
lv_obj_add_flag( ui_sine_wave_btn, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_sine_wave_btn, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_sine_wave_btn, lv_color_hex(0x393939), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_sine_wave_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_sine_wave_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_sine_wave_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_width(ui_sine_wave_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_spread(ui_sine_wave_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_color(ui_sine_wave_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_sine_wave_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_sine_wave_btn, lv_color_hex(0xFFF700), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_sine_wave_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_side(ui_sine_wave_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_text_color(ui_sine_wave_btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_text_opa(ui_sine_wave_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);

ui_Label3 = lv_label_create(ui_sine_wave_btn);
lv_obj_set_width( ui_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label3,"Sine");

ui_square_wave_btn = lv_btn_create(ui_Panel2);
lv_obj_set_height( ui_square_wave_btn, 45);
lv_obj_set_width( ui_square_wave_btn, lv_pct(30));
lv_obj_set_align( ui_square_wave_btn, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_square_wave_btn, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_square_wave_btn, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_square_wave_btn, lv_color_hex(0x393939), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_square_wave_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_square_wave_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_square_wave_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_width(ui_square_wave_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_spread(ui_square_wave_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_color(ui_square_wave_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_square_wave_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_square_wave_btn, lv_color_hex(0xFFF700), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_square_wave_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_side(ui_square_wave_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_text_color(ui_square_wave_btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_text_opa(ui_square_wave_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);

ui_Label1 = lv_label_create(ui_square_wave_btn);
lv_obj_set_width( ui_Label1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label1,"Square");

ui_triangle_wave_btn = lv_btn_create(ui_Panel2);
lv_obj_set_height( ui_triangle_wave_btn, 45);
lv_obj_set_width( ui_triangle_wave_btn, lv_pct(30));
lv_obj_set_align( ui_triangle_wave_btn, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_triangle_wave_btn, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_triangle_wave_btn, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_triangle_wave_btn, lv_color_hex(0x393939), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_triangle_wave_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_triangle_wave_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_triangle_wave_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_width(ui_triangle_wave_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_spread(ui_triangle_wave_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_color(ui_triangle_wave_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_triangle_wave_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_triangle_wave_btn, lv_color_hex(0xFFF700), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_triangle_wave_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_side(ui_triangle_wave_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_text_color(ui_triangle_wave_btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_text_opa(ui_triangle_wave_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);

ui_Label2 = lv_label_create(ui_triangle_wave_btn);
lv_obj_set_width( ui_Label2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label2,"Triangle");

ui_Label5 = lv_label_create(ui_Panel1);
lv_obj_set_width( ui_Label5, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label5, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label5, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label5,"Frequency");
lv_obj_set_style_text_color(ui_Label5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label5, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel3 = lv_obj_create(ui_Panel1);
lv_obj_set_width( ui_Panel3, lv_pct(100));
lv_obj_set_height( ui_Panel3, LV_SIZE_CONTENT);   /// 50
lv_obj_set_align( ui_Panel3, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel3,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_Panel3, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Panel3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel3, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel3, 5, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_freq_input = lv_textarea_create(ui_Panel3);
lv_obj_set_width( ui_freq_input, 97);
lv_obj_set_height( ui_freq_input, LV_SIZE_CONTENT);   /// 70
lv_obj_set_x( ui_freq_input, 23 );
lv_obj_set_y( ui_freq_input, 33 );
lv_obj_set_align( ui_freq_input, LV_ALIGN_CENTER );
lv_textarea_set_text(ui_freq_input,"10");
lv_textarea_set_one_line(ui_freq_input,true);
lv_obj_set_style_text_color(ui_freq_input, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_freq_input, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_freq_input, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_freq_input, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_freq_input, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_freq_input, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_freq_input, lv_color_hex(0x393839), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_freq_input, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_freq_input, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_freq_input, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_freq_input, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_freq_input, 5, LV_PART_MAIN| LV_STATE_DEFAULT);



ui_unit_hz_btn = lv_btn_create(ui_Panel3);
lv_obj_set_width( ui_unit_hz_btn, 45);
lv_obj_set_height( ui_unit_hz_btn, 45);
lv_obj_set_align( ui_unit_hz_btn, LV_ALIGN_CENTER );
lv_obj_add_state( ui_unit_hz_btn, LV_STATE_CHECKED );     /// States
lv_obj_add_flag( ui_unit_hz_btn, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_unit_hz_btn, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_unit_hz_btn, lv_color_hex(0x393939), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_unit_hz_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_unit_hz_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_unit_hz_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_width(ui_unit_hz_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_spread(ui_unit_hz_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_color(ui_unit_hz_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_unit_hz_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_unit_hz_btn, lv_color_hex(0xFFF700), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_unit_hz_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_side(ui_unit_hz_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_text_color(ui_unit_hz_btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_text_opa(ui_unit_hz_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);

ui_Label6 = lv_label_create(ui_unit_hz_btn);
lv_obj_set_width( ui_Label6, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label6, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label6, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label6,"Hz");

ui_unit_khz_btn = lv_btn_create(ui_Panel3);
lv_obj_set_width( ui_unit_khz_btn, 45);
lv_obj_set_height( ui_unit_khz_btn, 45);
lv_obj_set_align( ui_unit_khz_btn, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_unit_khz_btn, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_unit_khz_btn, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_unit_khz_btn, lv_color_hex(0x393939), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_unit_khz_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_unit_khz_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_unit_khz_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_width(ui_unit_khz_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_spread(ui_unit_khz_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_color(ui_unit_khz_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_unit_khz_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_unit_khz_btn, lv_color_hex(0xFFF700), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_unit_khz_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_side(ui_unit_khz_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_text_color(ui_unit_khz_btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_text_opa(ui_unit_khz_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);

ui_Label7 = lv_label_create(ui_unit_khz_btn);
lv_obj_set_width( ui_Label7, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label7, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label7, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label7,"kHz");

ui_unit_mhz_btn = lv_btn_create(ui_Panel3);
lv_obj_set_width( ui_unit_mhz_btn, 45);
lv_obj_set_height( ui_unit_mhz_btn, 45);
lv_obj_set_align( ui_unit_mhz_btn, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_unit_mhz_btn, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_unit_mhz_btn, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_unit_mhz_btn, lv_color_hex(0x393939), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_unit_mhz_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_unit_mhz_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_unit_mhz_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_width(ui_unit_mhz_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_spread(ui_unit_mhz_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_color(ui_unit_mhz_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_unit_mhz_btn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_unit_mhz_btn, lv_color_hex(0xFFF700), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_unit_mhz_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_side(ui_unit_mhz_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_text_color(ui_unit_mhz_btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_text_opa(ui_unit_mhz_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);

ui_Label8 = lv_label_create(ui_unit_mhz_btn);
lv_obj_set_width( ui_Label8, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label8, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label8, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label8,"MHz");

ui_Panel5 = lv_obj_create(ui_Panel1);
lv_obj_set_width( ui_Panel5, lv_pct(100));
lv_obj_set_height( ui_Panel5, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Panel5, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel5,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_Panel5, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel5, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Panel5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel5, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel5, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel6 = lv_obj_create(ui_Panel5);
lv_obj_set_width( ui_Panel6, lv_pct(50));
lv_obj_set_height( ui_Panel6, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Panel6, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel6,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_Panel6, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel6, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Panel6, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Panel6, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel6, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_Panel6, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel6, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel6, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel6, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel6, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel6, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel6, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel6, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label11 = lv_label_create(ui_Panel6);
lv_obj_set_width( ui_Label11, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label11, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label11, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label11,"Amplitude");
lv_obj_set_style_text_color(ui_Label11, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label11, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel7 = lv_obj_create(ui_Panel6);
lv_obj_set_width( ui_Panel7, lv_pct(100));
lv_obj_set_height( ui_Panel7, LV_SIZE_CONTENT);   /// 50
lv_obj_set_align( ui_Panel7, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel7,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_Panel7, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel7, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Panel7, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel7, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_Panel7, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel7, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel7, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel7, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel7, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel7, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_amplitude_input = lv_textarea_create(ui_Panel7);
lv_obj_set_width( ui_amplitude_input, 97);
lv_obj_set_height( ui_amplitude_input, LV_SIZE_CONTENT);   /// 70
lv_obj_set_x( ui_amplitude_input, 23 );
lv_obj_set_y( ui_amplitude_input, 33 );
lv_obj_set_align( ui_amplitude_input, LV_ALIGN_CENTER );
lv_textarea_set_text(ui_amplitude_input,"5");
lv_textarea_set_one_line(ui_amplitude_input,true);
lv_obj_set_style_text_color(ui_amplitude_input, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_amplitude_input, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_amplitude_input, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_amplitude_input, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_amplitude_input, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_amplitude_input, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_amplitude_input, lv_color_hex(0x393839), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_amplitude_input, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_amplitude_input, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_amplitude_input, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_amplitude_input, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_amplitude_input, 5, LV_PART_MAIN| LV_STATE_DEFAULT);



ui_Label12 = lv_label_create(ui_Panel7);
lv_obj_set_width( ui_Label12, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label12, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label12, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label12,"V");
lv_obj_set_style_text_color(ui_Label12, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label12, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label12, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel4 = lv_obj_create(ui_Panel5);
lv_obj_set_width( ui_Panel4, lv_pct(50));
lv_obj_set_height( ui_Panel4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Panel4, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel4,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_Panel4, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel4, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Panel4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel4, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel4, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label9 = lv_label_create(ui_Panel4);
lv_obj_set_width( ui_Label9, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label9, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label9, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label9,"Offset");
lv_obj_set_style_text_color(ui_Label9, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label9, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel8 = lv_obj_create(ui_Panel4);
lv_obj_set_width( ui_Panel8, lv_pct(100));
lv_obj_set_height( ui_Panel8, LV_SIZE_CONTENT);   /// 50
lv_obj_set_align( ui_Panel8, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel8,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_Panel8, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel8, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Panel8, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel8, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_Panel8, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel8, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel8, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel8, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel8, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel8, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_offset_input = lv_textarea_create(ui_Panel8);
lv_obj_set_width( ui_offset_input, 97);
lv_obj_set_height( ui_offset_input, LV_SIZE_CONTENT);   /// 70
lv_obj_set_x( ui_offset_input, 23 );
lv_obj_set_y( ui_offset_input, 33 );
lv_obj_set_align( ui_offset_input, LV_ALIGN_CENTER );
lv_textarea_set_text(ui_offset_input,"0");
lv_textarea_set_one_line(ui_offset_input,true);
lv_obj_set_style_text_color(ui_offset_input, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_offset_input, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_offset_input, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_offset_input, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_offset_input, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_offset_input, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_offset_input, lv_color_hex(0x393839), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_offset_input, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_offset_input, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_offset_input, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_offset_input, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_offset_input, 5, LV_PART_MAIN| LV_STATE_DEFAULT);



ui_Label10 = lv_label_create(ui_Panel8);
lv_obj_set_width( ui_Label10, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label10, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label10, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label10,"V");
lv_obj_set_style_text_color(ui_Label10, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label10, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_output_btn = lv_btn_create(ui_Panel1);
lv_obj_set_height( ui_output_btn, 45);
lv_obj_set_width( ui_output_btn, lv_pct(100));
lv_obj_set_align( ui_output_btn, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_output_btn, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_output_btn, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_border_width(ui_output_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_output_btn, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_width(ui_output_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_spread(ui_output_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_output_btn, lv_color_hex(0x02C600), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_output_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_color(ui_output_btn, lv_color_hex(0xFFF600), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_border_opa(ui_output_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_width(ui_output_btn, 3, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_side(ui_output_btn, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_shadow_width(ui_output_btn, 0, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_shadow_spread(ui_output_btn, 0, LV_PART_MAIN| LV_STATE_CHECKED);

ui_Label13 = lv_label_create(ui_output_btn);
lv_obj_set_width( ui_Label13, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label13, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label13, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label13,"OUTPUT");
lv_obj_set_style_text_font(ui_Label13, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_keypad = lv_keyboard_create(ui_Index);
lv_keyboard_set_mode(ui_keypad,LV_KEYBOARD_MODE_NUMBER);
lv_obj_set_width( ui_keypad, lv_pct(100));
lv_obj_set_flex_grow( ui_keypad, 1);
lv_obj_set_align( ui_keypad, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_keypad, LV_OBJ_FLAG_HIDDEN );   /// Flags


}
