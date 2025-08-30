#pragma once
#include "driver/spi_master.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EPD_WIDTH  264
#define EPD_HEIGHT 176

typedef struct {
    spi_device_handle_t spi;
    int pin_dc;
    int pin_rst;
    int pin_busy;
    bool busy_is_high; // true: BUSY=1 means busy; false: BUSY=0 means busy
} epd_handle_t;

typedef struct {
    int pin_mosi;
    int pin_sclk;
    int pin_cs;
    int pin_dc;
    int pin_rst;
    int pin_busy;
    bool busy_is_high;
    spi_host_device_t host;   // SPI2_HOST on classic ESP32
    int max_transfer_sz;      // bytes; suggest EPD_WIDTH*EPD_HEIGHT/8
    int clk_hz;               // 4–10 MHz is safe
} epd_config_t;

// life-cycle
esp_err_t epd_create(const epd_config_t *cfg, epd_handle_t *out);
void      epd_destroy(epd_handle_t *epd);

// panel control
esp_err_t epd_reset(epd_handle_t *epd);
esp_err_t epd_init(epd_handle_t *epd);         // one-time init
esp_err_t epd_clear(epd_handle_t *epd, uint8_t white /*0xFF*/);

// draw (1bpp packed MSB→left)
esp_err_t epd_draw_full(epd_handle_t *epd, const uint8_t *framebuf /*(W*H/8)*/);

// optional area draw (simple windowed write + full refresh)
esp_err_t epd_draw_area(epd_handle_t *epd, int x, int y, int w, int h, const uint8_t *bits);

// refresh & sleep
esp_err_t epd_refresh_full(epd_handle_t *epd);
esp_err_t epd_sleep(epd_handle_t *epd);

#ifdef __cplusplus
}
#endif

