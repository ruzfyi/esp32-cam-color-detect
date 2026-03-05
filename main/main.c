#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "freertos/idf_additions.h"
#include "jpeg_decoder.h"
#include "portmacro.h"

// define pins for esp32 cam
#define CAM_PIN_PWDN    32 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// decoder definitions
#define IMG_W 320
#define IMG_H 240
#define PIXEL_COUNT (IMG_W * IMG_H)

static const char *TAG = "CAMERAPROG";

static camera_config_t camera_config = {
      .pin_pwdn  = CAM_PIN_PWDN,
      .pin_reset = CAM_PIN_RESET,
      .pin_xclk = CAM_PIN_XCLK,
      .pin_sccb_sda = CAM_PIN_SIOD,
      .pin_sccb_scl = CAM_PIN_SIOC,

      .pin_d7 = CAM_PIN_D7,
      .pin_d6 = CAM_PIN_D6,
      .pin_d5 = CAM_PIN_D5,
      .pin_d4 = CAM_PIN_D4,
      .pin_d3 = CAM_PIN_D3,
      .pin_d2 = CAM_PIN_D2,
      .pin_d1 = CAM_PIN_D1,
      .pin_d0 = CAM_PIN_D0,
      .pin_vsync = CAM_PIN_VSYNC,
      .pin_href = CAM_PIN_HREF,
      .pin_pclk = CAM_PIN_PCLK,

      .xclk_freq_hz = 20000000,
      .ledc_timer = LEDC_TIMER_0,
      .ledc_channel = LEDC_CHANNEL_0,

      .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
      .frame_size = FRAMESIZE_QVGA,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

      .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
      .fb_location = CAMERA_FB_IN_PSRAM,
      .fb_count = 1, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
      .grab_mode = CAMERA_GRAB_WHEN_EMPTY//CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

// struct for the hsv ranges file
typedef struct {
      uint8_t h_min, h_max;
      uint8_t s_min, s_max;
      uint8_t v_min, v_max;
} hsv_threshold_t;

// h_min, h_max, s_min, s_max, v_min, v_max
hsv_threshold_t thresholds[5] = {
      { 0,    10,   100,  255,   50,  255 }, // red1
      { 170,  180,  100,  255,   50,  255 }, // red2
      { 40,   80,   100,  255,  100,  255 }, // green
      { 100,  120,  100,  255,  100,  255 }, // blue
      { 115,  165,  100,  255,  100,  255 } // purple
};

static esp_err_t camera_init() {
      //initialize the camera
      esp_err_t err = esp_camera_init(&camera_config);
      if (err != ESP_OK) {
            ESP_LOGE(TAG, "Camera Init Failed");
            return err;
      }

      return ESP_OK;
}

static void rgb888_to_hsv(uint8_t r, uint8_t g, uint8_t b, uint8_t *h, uint8_t *s, uint8_t *v) {
      uint8_t min = r < g ? (r < b ? r : b) : (g < b ? g : b);
      uint8_t max = r > g ? (r > b ? r : b) : (g > b ? g : b);
      uint8_t delta = max - min;

      *v = max;
      if (max == 0 || delta == 0) {
            *s = 0;
            *h = 0;
            return;
      }

      *s = (uint16_t)255 * delta / max;

      int32_t hue;
      if (max == r) hue = 0 + 60 * (g - b) / delta;
      else if (max == g) hue = 120 + 60 * (b - r) / delta;
      else hue = 240 + 60 * (r - g) / delta;

      if (hue < 0) hue += 360;
      *h = hue / 2;
}

void app_main() {
      if (camera_init() != ESP_OK) {
            ESP_LOGE(TAG, "Camera failed to initialize");
            return;
      }

      // warm up the camera
      for (int i = 0; i < 2; i++) {
            camera_fb_t * fb = esp_camera_fb_get();
            if (fb) {
                  esp_camera_fb_return(fb);
                  vTaskDelay(200);
            }
      }

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) return;

      esp_jpeg_image_output_t out_info;
      esp_jpeg_image_cfg_t config = {
            .indata = fb->buf,
            .indata_size = fb->len,
            .out_format = JPEG_IMAGE_FORMAT_RGB888,
            .out_scale = JPEG_IMAGE_SCALE_0,
            .flags = {
                  .swap_color_bytes = 0
            }
      };

      if (esp_jpeg_get_image_info(&config, &out_info) != ESP_OK) {
            ESP_LOGE(TAG, "JPEG info failed");
            esp_camera_fb_return(fb);
            return;
      }

      uint8_t *rgb_buf = (uint8_t *)heap_caps_malloc(out_info.output_len, MALLOC_CAP_SPIRAM);
      if (!rgb_buf) {
            ESP_LOGE(TAG, "SPIRAM allocation failed (%d bytes)", out_info.output_len);
            esp_camera_fb_return(fb);
            return;
      }

      config.outbuf = rgb_buf;
      config.outbuf_size = out_info.output_len;

      if (esp_jpeg_decode(&config, &out_info) == ESP_OK) {
            uint32_t count_r = 0, count_g = 0, count_b = 0, count_p = 0;
            uint32_t total_pixels = out_info.width * out_info.height;

            for (uint32_t i = 0; i < out_info.output_len; i += 3) {
                  uint8_t h, s, v;
                  rgb888_to_hsv(rgb_buf[i], rgb_buf[i+1], rgb_buf[i+2], &h, &s, &v);

                  if ((h >= thresholds[0].h_min && h <= thresholds[0].h_max && s >= thresholds[0].s_min && v >= thresholds[0].v_min) ||
                  (h >= thresholds[1].h_min && h <= thresholds[1].h_max && s >= thresholds[1].s_min && v >= thresholds[1].v_min)) {
                        count_r++;
                  }
                  else if (h >= thresholds[2].h_min && h <= thresholds[2].h_max && s >= thresholds[2].s_min && v >= thresholds[2].v_min) {
                        count_g++;
                  }
                  else if (h >= thresholds[3].h_min && h <= thresholds[3].h_max && s >= thresholds[3].s_min && v >= thresholds[3].v_min) {
                        count_b++;
                  }
                  else if (h >= thresholds[4].h_min && h <= thresholds[4].h_max && s >= thresholds[4].s_min && v >= thresholds[4].v_min) {
                        count_p++;
                  }
            }

            float red_per, blue_per, green_per, purple_per;
            red_per = (count_r * 100.0f) / total_pixels;
            green_per = (count_g * 100.0f) / total_pixels;
            blue_per = (count_b * 100.0f) / total_pixels;
            purple_per = (count_p * 100.0f) / total_pixels;

            ESP_LOGI(TAG, "Results\n");
            ESP_LOGI(TAG, "Red: %.2f%%\n", red_per);
            ESP_LOGI(TAG, "Blue: %.2f%%\n", blue_per);
            ESP_LOGI(TAG, "Green: %.2f%%\n", green_per);
            ESP_LOGI(TAG, "Purple: %.2f%%\n", purple_per);

            char *final_res;

            if (red_per > blue_per) {
                  if (red_per > green_per) {
                        if (red_per > purple_per) {
                              final_res = "RED";
                        } else {
                              final_res = "PURPLE";
                        }
                  } else {
                        if (green_per > purple_per) {
                              final_res = "GREEN";
                        } else {
                              final_res = "BLUE";
                        }
                  }
            } else {
                  if (blue_per > green_per) {
                        if (blue_per > purple_per) {
                              final_res = "BLUE";
                        } else {
                              final_res = "PURPLE";
                        }
                  } else {
                        if (green_per > purple_per) {
                              final_res = "GREEN";
                        } else {
                              final_res = "RED";
                        }
                  }
            }

            if (purple_per > 0.2f) {
                  final_res = "PURPLE";
            }

            ESP_LOGI(TAG, "RPISEND: %s", final_res); // output final response via Serial for RPI to interpret
      }

      heap_caps_free(rgb_buf);
      esp_camera_fb_return(fb);


}
