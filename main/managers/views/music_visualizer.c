#include "managers/views/music_visualizer.h"
#include "managers/views/main_menu_screen.h"
#include "vendor/drivers/ws_audio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <lvgl.h>
#include <math.h>
#include "esp_log.h"
#include "gui/screen_layout.h"
#include "gui/lvgl_safe.h"

#define ANIMATION_INTERVAL_MS 5 // Approximately 30 FPS

static const char *TAG = "MusicVisualizer";

lv_timer_t *animation_timer = NULL;

typedef struct {
  int bars[NUM_BARS]; // Amplitude data for each bar
} AmplitudeData;

MusicVisualizerView view;
lv_obj_t *root;
QueueHandle_t amplitudeQueue;

int target_amplitudes[NUM_BARS] = {0};
int current_amplitudes[NUM_BARS] = {0};

static void animation_timer_callback(lv_timer_t *timer);

void handle_hardware_input_music_callback(InputEvent *event) {
  if (event->type == INPUT_TYPE_TOUCH) {
    ESP_LOGI(TAG, "Touch event");
    display_manager_switch_view(&main_menu_view);
  } else if (event->type == INPUT_TYPE_JOYSTICK) {
    ESP_LOGI(TAG, "Joystick event");

    int button = event->data.joystick_index;
    if (button == 1) {
      display_manager_switch_view(&main_menu_view);
    }
  } else if (event->type == INPUT_TYPE_KEYBOARD){ 
    ESP_LOGW(TAG, "keyboard event");
    uint8_t key = event->data.key_value;
    if (key == 27 || key == '`'){
    display_manager_switch_view(&main_menu_view);
    }
#ifdef CONFIG_USE_ENCODER
  } else if (event->type == INPUT_TYPE_EXIT_BUTTON) {
    ESP_LOGI(TAG, "IO6 exit button pressed, returning to main menu");
    display_manager_switch_view(&main_menu_view);
#endif
  }
}

void get_music_visualizer_callback(void **callback) {
  *callback = music_visualizer_view.input_callback;
}

View music_visualizer_view = {
    .root = NULL,
    .create = music_visualizer_view_create,
    .destroy = music_visualizer_destroy,
    .input_callback = handle_hardware_input_music_callback,
    .name = "Music Visualizer",
    .get_hardwareinput_callback = get_music_visualizer_callback};

void music_visualizer_view_create() {
  display_manager_fill_screen(lv_color_black());

  root = gui_screen_create_root(NULL, (LV_VER_RES > 320 ? "Rave Mode" : "Rave"), lv_color_black(), LV_OPA_COVER);
  music_visualizer_view.root = root;
  lv_obj_t *content = gui_screen_create_content(root, GUI_STATUS_BAR_HEIGHT);
  lv_obj_set_style_pad_column(content, LV_HOR_RES / 24, 0);

  const lv_font_t *track_label_font;
  const lv_font_t *artist_label_font;

  if (LV_HOR_RES <= 128) {
    track_label_font = &lv_font_montserrat_12;
    artist_label_font = &lv_font_montserrat_10;
  } else if (LV_HOR_RES <= 240) {
    track_label_font = &lv_font_montserrat_16;
    artist_label_font = &lv_font_montserrat_12;
  } else {
    track_label_font = &lv_font_montserrat_24;
    artist_label_font = &lv_font_montserrat_16;
  }

  int label_x_offset = 10;
  int label_y_offset = LV_VER_RES / 8;
  int avail_w = LV_HOR_RES - 20;  // 10px padding each side
  int bar_spacing = avail_w / NUM_BARS;  // even distribution across width
  int gap = (bar_spacing > 6) ? 4 : 2;   // gap between bars
  int bar_width = bar_spacing - gap;
  if (bar_width < 2) bar_width = 2;
  /* Centre the bar group horizontally */
  int total_w = bar_spacing * NUM_BARS;
  int bar_x_start = (LV_HOR_RES - total_w) / 2;
  int bar_y_offset = LV_VER_RES / 4;

  view.track_label = lv_label_create(content);
  lv_label_set_text(view.track_label, "Ghost ESP");
  lv_obj_set_style_text_font(view.track_label, track_label_font, LV_PART_MAIN);
  lv_obj_set_style_text_color(view.track_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(view.track_label, LV_ALIGN_BOTTOM_LEFT, label_x_offset,
               -label_y_offset);

  view.artist_label = lv_label_create(content);
  lv_label_set_text(view.artist_label, "Spooky");
  lv_obj_set_style_text_font(view.artist_label, artist_label_font,
                             LV_PART_MAIN);
  lv_obj_set_style_text_color(view.artist_label, lv_color_white(),
                              LV_PART_MAIN);
  lv_obj_align_to(view.artist_label, view.track_label, LV_ALIGN_OUT_BOTTOM_LEFT,
                  0, lv_font_get_line_height(track_label_font) / 4);

  for (int i = 0; i < NUM_BARS; i++) {
    view.bars[i] = lv_obj_create(content);
    lv_obj_remove_style_all(view.bars[i]);
    lv_obj_clear_flag(view.bars[i], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(view.bars[i], bar_width, 1);
    lv_obj_align(view.bars[i], LV_ALIGN_BOTTOM_LEFT,
                 bar_x_start + (bar_spacing * i), -bar_y_offset);

    lv_obj_set_style_radius(view.bars[i], 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(view.bars[i], LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(view.bars[i], lv_color_make(147, 112, 219),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(view.bars[i], lv_color_make(147, 112, 219),
                                   LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(view.bars[i], LV_GRAD_DIR_VER, LV_PART_MAIN);
  }

  amplitudeQueue = xQueueCreate(10, sizeof(AmplitudeData));
  animation_timer =
      lv_timer_create(animation_timer_callback, ANIMATION_INTERVAL_MS, NULL);

  /* Start microphone capture → amplitude data will flow into amplitudeQueue */
  if (ws_audio_init() == ESP_OK) {
    ws_audio_start();
  }
}

static void animation_timer_callback(lv_timer_t *timer) {
  AmplitudeData amplitudeData;
  bool dataAvailable =
      xQueueReceive(amplitudeQueue, &amplitudeData, 0) == pdTRUE;

  if (dataAvailable) {
    for (int i = 0; i < NUM_BARS; i++) {
      target_amplitudes[i] = amplitudeData.bars[i];
    }
  } else {
    /* Self-animating demo mode: randomly pick new target heights so the
     * visualizer looks alive even without Bluetooth audio data. */
    for (int i = 0; i < NUM_BARS; i++) {
      if (rand() % 6 == 0) { /* ~17 % chance per tick per bar */
        int max_h = (LV_VER_RES - GUI_STATUS_BAR_HEIGHT) / 2;
        target_amplitudes[i] = 10 + rand() % (max_h > 10 ? max_h : 20);
      }
      /* Gravity: targets slowly decay toward zero */
      if (target_amplitudes[i] > 2) target_amplitudes[i] -= 1;
    }
  }

  /* Smooth interpolation toward targets */
  for (int i = 0; i < NUM_BARS; i++) {
    int diff = target_amplitudes[i] - current_amplitudes[i];
    current_amplitudes[i] += diff / 4 + (diff > 0 ? 1 : (diff < 0 ? -1 : 0));
    if (current_amplitudes[i] < 1) current_amplitudes[i] = 1;
    lv_obj_set_height(view.bars[i], current_amplitudes[i]);
  }


}

void music_visualizer_view_update(const uint8_t *amplitudes,
                                  const char *track_name,
                                  const char *artist_name) {

  if (music_visualizer_view.root) {
    if (strcmp(lv_label_get_text(view.track_label), track_name) != 0) {
      lv_label_set_text(view.track_label, track_name);
    }
    if (strcmp(lv_label_get_text(view.artist_label), artist_name) != 0) {
      lv_label_set_text(view.artist_label, artist_name);
    }

    AmplitudeData amplitudeData;
    for (int i = 0; i < NUM_BARS; i++) {
      amplitudeData.bars[i] = amplitudes[i];
    }
    xQueueSend(amplitudeQueue, &amplitudeData, portMAX_DELAY);
  }
}

void music_visualizer_destroy(void) {

  ws_audio_stop();

  lvgl_timer_del_safe(&animation_timer);

  lvgl_obj_del_safe(&root);
  music_visualizer_view.root = NULL;
  if (amplitudeQueue) {
    vQueueDelete(amplitudeQueue);
    amplitudeQueue = NULL;
  }
}