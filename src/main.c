#include "epd_ssd1681.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "main";

void app_main(void) {
  epd_config_t cfg = {
      .pin_mosi = EPD_PIN_MOSI,
      .pin_sclk = EPD_PIN_SCLK,
      .pin_cs = EPD_PIN_CS,
      .pin_dc = EPD_PIN_DC,
      .pin_rst = EPD_PIN_RST,
      .pin_busy = EPD_PIN_BUSY,
      .busy_is_high = EPD_BUSY_IS_HIGH, // 1 for most Waveshare 2.7" units
      .host = SPI2_HOST,
      .clk_hz = 10 * 1000 * 1000,
      .max_transfer_sz = EPD_WIDTH * EPD_HEIGHT / 8 + 16,
  };

  epd_handle_t epd;
  ESP_ERROR_CHECK(epd_create(&cfg, &epd));
  ESP_ERROR_CHECK(epd_init(&epd));

  // make a white framebuffer
  size_t fb_sz = (EPD_WIDTH * EPD_HEIGHT) / 8;
  uint8_t *fb = malloc(fb_sz);
  memset(fb, 0xFF, fb_sz); // white

  // draw a black rect 20,20..120,60 into fb (MSB = leftmost)
  for (int y = 20; y < 60; ++y) {
    for (int x = 20; x < 120; ++x) {
      size_t bit = (size_t)y * EPD_WIDTH + x;
      fb[bit >> 3] &= (uint8_t)~(1 << (7 - (bit & 7)));
    }
  }

  ESP_ERROR_CHECK(epd_draw_full(&epd, fb));
  ESP_ERROR_CHECK(epd_refresh_full(&epd));

  ESP_LOGI(TAG, "drawn rectangle; sleeping in 2s");
  vTaskDelay(pdMS_TO_TICKS(2000));
  ESP_ERROR_CHECK(epd_sleep(&epd));
  free(fb);
}
