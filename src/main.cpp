#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <FT6336.h>
#include <driver/i2s.h>

#include "gui/ui.h"

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

#define LCD_BL_PIN (3)

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

#define I2C_SDA_PIN (15)
#define I2C_SCL_PIN (16)

TFT_eSPI tft = TFT_eSPI(); /* TFT instance */

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t *indev_driver, lv_indev_data_t * data) {
  uint8_t touchPoint = touch.read((uint16_t*)(&data->point.x), (uint16_t*)(&data->point.y));
  if (touchPoint > 0) {
    // Serial.printf("X: %d\tY: %d\n", data->point.x, data->point.y);
  }
  data->state = touchPoint > 0 ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

void waveform_click_cb(lv_event_t * e) {
  lv_obj_t * target = lv_event_get_target(e);

  lv_obj_t * waveform_select_btn[] = {
    ui_sine_wave_btn,
    ui_square_wave_btn,
    ui_triangle_wave_btn,
  };
  for (uint8_t i=0;i<(sizeof(waveform_select_btn) / sizeof(lv_obj_t*));i++) {
    lv_obj_clear_state(waveform_select_btn[i], LV_STATE_CHECKED);
  }

  lv_obj_add_state(target, LV_STATE_CHECKED);
  
  if (target == ui_sine_wave_btn) {
    lv_img_set_src(ui_wave_form_img, &ui_img_sine_png);
  } else if (target == ui_square_wave_btn) {
    lv_img_set_src(ui_wave_form_img, &ui_img_square_png);
  } else if (target == ui_triangle_wave_btn) {
    lv_img_set_src(ui_wave_form_img, &ui_img_tri_png);
  }
}

void unit_click_cb(lv_event_t * e) {
  lv_obj_t * target = lv_event_get_target(e);

  lv_obj_t * unit_select_btn[] = {
    ui_unit_hz_btn,
    ui_unit_khz_btn,
    ui_unit_mhz_btn,
  };
  for (uint8_t i=0;i<(sizeof(unit_select_btn) / sizeof(lv_obj_t*));i++) {
    lv_obj_clear_state(unit_select_btn[i], LV_STATE_CHECKED);
  }

  lv_obj_add_state(target, LV_STATE_CHECKED);
}

void output_click_cb(lv_event_t * e) {
  bool output_enable = lv_obj_has_state(ui_output_btn, LV_STATE_CHECKED);
  if (output_enable) {
    lv_label_set_text_fmt(ui_output_log_label, 
      "Waveform: %s\nFrequency: %.03f %s\nVp-p: %.03fV",
      lv_obj_has_state(ui_sine_wave_btn, LV_STATE_CHECKED) ? "Sine" : lv_obj_has_state(ui_square_wave_btn, LV_STATE_CHECKED) ? "Square" : "Triangle",
      atof(lv_textarea_get_text(ui_freq_input)),
      lv_obj_has_state(ui_unit_hz_btn, LV_STATE_CHECKED) ? "Hz" : lv_obj_has_state(ui_unit_khz_btn, LV_STATE_CHECKED) ? "kHz" : "MHz",
      atof(lv_textarea_get_text(ui_amplitude_input))
    );
  } else {
    lv_label_set_text(ui_output_log_label, "None");
  }
}

// Sound
#define I2S_DOUT      47
#define I2S_BCLK      48
#define I2S_LRC       45
i2s_port_t i2s_num = I2S_NUM_0; // i2s port number

uint16_t beep_sound_wave[800]; // 16 kHz sample rate

void beep() {
  size_t out_bytes = 0;
  i2s_write(i2s_num, beep_sound_wave, sizeof(beep_sound_wave), &out_bytes, 100);
}

void my_inp_feedback(lv_indev_drv_t *indev_driver, uint8_t event) {
    if((event == LV_EVENT_CLICKED) || (event == LV_EVENT_KEY)) {
        beep();
    }
}

void setup() {
  Serial.begin( 115200 ); /* prepare for possible serial debug */

  // Gen sound beep
  for (uint16_t i=0;i<800;i++) {
    beep_sound_wave[i] = (i % 16) > 8 ? 0x3FFF : 0x0000;
    // Serial.println(beep_sound_wave[i]);
  }

  static const i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  static i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // Interrupt level 1, default 0
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false,
      .tx_desc_auto_clear = true
  };

  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  i2s_set_pin(i2s_num, &pin_config);


  lv_init();

  tft.begin();          /* TFT init */
  tft.setRotation( 1 ); /* Landscape orientation, flipped */
    // tft.invertDisplay(false);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400E3);
  touch.init();

  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 10 );

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  indev_drv.feedback_cb = my_inp_feedback;
  lv_indev_drv_register(&indev_drv);
    
  ui_init();

  lv_obj_t * waveform_select_btn[] = {
    ui_sine_wave_btn,
    ui_square_wave_btn,
    ui_triangle_wave_btn,
  };
  for (uint8_t i=0;i<(sizeof(waveform_select_btn) / sizeof(lv_obj_t*));i++) {
    lv_obj_add_event_cb(waveform_select_btn[i], waveform_click_cb, LV_EVENT_CLICKED, NULL);
  }
    
  lv_obj_t * unit_select_btn[] = {
    ui_unit_hz_btn,
    ui_unit_khz_btn,
    ui_unit_mhz_btn,
  };
  for (uint8_t i=0;i<(sizeof(unit_select_btn) / sizeof(lv_obj_t*));i++) {
    lv_obj_add_event_cb(unit_select_btn[i], unit_click_cb, LV_EVENT_CLICKED, NULL);
  }

  lv_obj_add_event_cb(ui_output_btn, output_click_cb, LV_EVENT_CLICKED, NULL);
  lv_label_set_text(ui_output_log_label, "None");

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);
}
