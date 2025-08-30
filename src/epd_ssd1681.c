#include "epd_ssd1681.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include <string.h>

#define TAG "epd"

// ===== SSD1680/SSD1681 commands (common set) =====
#define CMD_DRIVER_OUTPUT_CONTROL 0x01
#define CMD_GATE_DRIVING_VOLTAGE 0x03 // (not strictly required)
#define CMD_SOFTSTART 0x0C
#define CMD_DEEP_SLEEP 0x10
#define CMD_DATA_ENTRY_MODE 0x11
#define CMD_SW_RESET 0x12
#define CMD_TEMPERATURE_SENSOR_CONTROL 0x18
#define CMD_MASTER_ACTIVATION 0x20
#define CMD_DISPLAY_UPDATE_CONTROL_2 0x22
#define CMD_WRITE_RAM_BW 0x24
#define CMD_WRITE_RAM_RED 0x26 // for tri-color panels; unused here
#define CMD_VCOM_SENSE_DURING_VCI_DET                                          \
  0x2F // not used; BUSY behavior described in ds
#define CMD_SET_VCOM 0x2C
#define CMD_WRITE_LUT 0x32
#define CMD_SET_RAM_X_START_END 0x44
#define CMD_SET_RAM_Y_START_END 0x45
#define CMD_BORDER_WAVEFORM_CONTROL 0x3C
#define CMD_DUMMY_LINE_PERIOD 0x3A
#define CMD_GATE_LINE_WIDTH 0x3B
#define CMD_SET_RAM_X_COUNTER 0x4E
#define CMD_SET_RAM_Y_COUNTER 0x4F

// ===== helpers =====
static inline void gpio_set(int pin, int level) { gpio_set_level(pin, level); }
static inline int gpio_get(int pin) { return gpio_get_level(pin); }
static inline void delay_ms(int ms) { esp_rom_delay_us(ms * 1000); }

static void wait_busy(epd_handle_t *epd) {
  // Most SSD1680/1681 boards: BUSY=1 while busy, BUSY=0 when ready. Datasheets
  // & vendor docs note this. If your board is inverted, set busy_is_high=false.
  // :contentReference[oaicite:1]{index=1}
  const int active = epd->busy_is_high ? 1 : 0;
  while (gpio_get(epd->pin_busy) == active) {
    delay_ms(5);
  }
}

static esp_err_t tx_bytes(epd_handle_t *epd, const uint8_t *data, size_t len,
                          int dc_level) {
  gpio_set(epd->pin_dc, dc_level);
  spi_transaction_t t = {0};
  t.length = len * 8;
  t.tx_buffer = data;
  return spi_device_transmit(epd->spi, &t);
}
static esp_err_t cmd1(epd_handle_t *epd, uint8_t cmd) {
  return tx_bytes(epd, &cmd, 1, 0);
}
static esp_err_t data(epd_handle_t *epd, const void *buf, size_t len) {
  return tx_bytes(epd, (const uint8_t *)buf, len, 1);
}
static esp_err_t data1(epd_handle_t *epd, uint8_t v) {
  return tx_bytes(epd, &v, 1, 1);
}

// ===== public =====
esp_err_t epd_create(const epd_config_t *cfg, epd_handle_t *out) {
  if (!cfg || !out)
    return ESP_ERR_INVALID_ARG;

  // SPI bus
  spi_bus_config_t buscfg = {
      .mosi_io_num = cfg->pin_mosi,
      .miso_io_num = -1,
      .sclk_io_num = cfg->pin_sclk,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = cfg->max_transfer_sz
                             ? cfg->max_transfer_sz
                             : (EPD_WIDTH * EPD_HEIGHT / 8 + 16),
  };
  ESP_ERROR_CHECK(spi_bus_initialize(
      cfg->host, &buscfg,
      SPI_DMA_CH_AUTO)); // uses DMA if possible
                         // :contentReference[oaicite:2]{index=2}

  // SPI device (CS handled by driver)
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = cfg->clk_hz ? cfg->clk_hz : 10 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = cfg->pin_cs,
      .queue_size = 8,
      .flags = SPI_DEVICE_HALFDUPLEX,
  };
  spi_device_handle_t spi;
  ESP_ERROR_CHECK(spi_bus_add_device(cfg->host, &devcfg, &spi));

  // GPIOs
  gpio_config_t io = {.mode = GPIO_MODE_OUTPUT,
                      .pin_bit_mask =
                          (1ULL << cfg->pin_dc) | (1ULL << cfg->pin_rst)};
  ESP_ERROR_CHECK(gpio_config(&io));
  gpio_config_t ib = {.mode = GPIO_MODE_INPUT,
                      .pin_bit_mask = (1ULL << cfg->pin_busy)};
  ESP_ERROR_CHECK(gpio_config(&ib));

  out->spi = spi;
  out->pin_dc = cfg->pin_dc;
  out->pin_rst = cfg->pin_rst;
  out->pin_busy = cfg->pin_busy;
  out->busy_is_high = cfg->busy_is_high;
  return ESP_OK;
}

void epd_destroy(epd_handle_t *epd) {
  if (!epd)
    return;
  spi_bus_remove_device(epd->spi);
  // (bus free omitted; do it if this is your only device)
}

esp_err_t epd_reset(epd_handle_t *epd) {
  gpio_set(epd->pin_rst, 0);
  delay_ms(10);
  gpio_set(epd->pin_rst, 1);
  delay_ms(10);
  return ESP_OK;
}

esp_err_t epd_init(epd_handle_t *epd) {
  ESP_ERROR_CHECK(epd_reset(epd));

  // Soft reset
  ESP_ERROR_CHECK(cmd1(epd, CMD_SW_RESET));
  wait_busy(epd);

  // Driver output control: (EPD_HEIGHT-1) LSB, MSB, GD/SM/TB=0
  uint8_t doc[] = {(uint8_t)((EPD_HEIGHT - 1) & 0xFF),
                   (uint8_t)(((EPD_HEIGHT - 1) >> 8) & 0xFF), 0x00};
  ESP_ERROR_CHECK(cmd1(epd, CMD_DRIVER_OUTPUT_CONTROL));
  ESP_ERROR_CHECK(data(epd, doc, sizeof(doc)));

  // Data entry mode: X inc, Y inc (0x03)  :contentReference[oaicite:3]{index=3}
  ESP_ERROR_CHECK(cmd1(epd, CMD_DATA_ENTRY_MODE));
  ESP_ERROR_CHECK(data1(epd, 0x03));

  // Set RAM X (in bytes!) [0 .. (W/8-1)]
  ESP_ERROR_CHECK(cmd1(epd, CMD_SET_RAM_X_START_END));
  uint8_t xse[] = {0x00, (uint8_t)((EPD_WIDTH / 8) - 1)};
  ESP_ERROR_CHECK(data(epd, xse, sizeof(xse)));

  // Set RAM Y [0 .. (H-1)]
  ESP_ERROR_CHECK(cmd1(epd, CMD_SET_RAM_Y_START_END));
  uint16_t y_end = EPD_HEIGHT - 1;
  uint8_t yse[] = {0x00, 0x00, (uint8_t)(y_end & 0xFF), (uint8_t)(y_end >> 8)};
  ESP_ERROR_CHECK(data(epd, yse, sizeof(yse)));

  // Border waveform (typical 0x05) & VCOM (typical 0x26)
  ESP_ERROR_CHECK(cmd1(epd, CMD_BORDER_WAVEFORM_CONTROL));
  ESP_ERROR_CHECK(data1(epd, 0x05));
  ESP_ERROR_CHECK(cmd1(epd, CMD_SET_VCOM));
  ESP_ERROR_CHECK(data1(epd, 0x26));

  // Timing (dummy line & gate width): common working values
  ESP_ERROR_CHECK(cmd1(epd, CMD_DUMMY_LINE_PERIOD));
  ESP_ERROR_CHECK(data1(epd, 0x1A));
  ESP_ERROR_CHECK(cmd1(epd, CMD_GATE_LINE_WIDTH));
  ESP_ERROR_CHECK(data1(epd, 0x08));

  // Soft start (recommended sequence)  :contentReference[oaicite:4]{index=4}
  ESP_ERROR_CHECK(cmd1(epd, CMD_SOFTSTART));
  uint8_t ss[] = {0xD7, 0xD6, 0x9D};
  ESP_ERROR_CHECK(data(epd, ss, sizeof(ss)));

  // (Optional) Write LUT here via CMD_WRITE_LUT if you need a custom waveform.
  // Most Waveshare modules work with OTP LUT for a full refresh, so we skip it
  // for minimal bring-up.  :contentReference[oaicite:5]{index=5}
  return ESP_OK;
}

static esp_err_t set_ram_ptr(epd_handle_t *epd, uint8_t x_bytes, uint16_t y) {
  ESP_ERROR_CHECK(cmd1(epd, CMD_SET_RAM_X_COUNTER));
  ESP_ERROR_CHECK(data1(epd, x_bytes));
  ESP_ERROR_CHECK(cmd1(epd, CMD_SET_RAM_Y_COUNTER));
  uint8_t yc[] = {(uint8_t)(y & 0xFF), (uint8_t)(y >> 8)};
  ESP_ERROR_CHECK(data(epd, yc, sizeof(yc)));
  return ESP_OK;
}

esp_err_t epd_clear(epd_handle_t *epd, uint8_t white) {
  // Set full window
  ESP_ERROR_CHECK(cmd1(epd, CMD_SET_RAM_X_START_END));
  uint8_t xse[] = {0x00, (uint8_t)((EPD_WIDTH / 8) - 1)};
  ESP_ERROR_CHECK(data(epd, xse, sizeof(xse)));
  ESP_ERROR_CHECK(cmd1(epd, CMD_SET_RAM_Y_START_END));
  uint16_t y_end = EPD_HEIGHT - 1;
  uint8_t yse[] = {0x00, 0x00, (uint8_t)(y_end & 0xFF), (uint8_t)(y_end >> 8)};
  ESP_ERROR_CHECK(data(epd, yse, sizeof(yse)));

  ESP_ERROR_CHECK(set_ram_ptr(epd, 0x00, 0x0000));
  ESP_ERROR_CHECK(cmd1(epd, CMD_WRITE_RAM_BW));

  // stream white pixels (0xFF = white for most bw panels)
  const size_t n = (EPD_WIDTH * EPD_HEIGHT) / 8;
  // send in chunks to avoid giant transactions
  uint8_t chunk[256];
  memset(chunk, white, sizeof(chunk));
  size_t left = n;
  while (left) {
    size_t c = left > sizeof(chunk) ? sizeof(chunk) : left;
    ESP_ERROR_CHECK(data(epd, chunk, c));
    left -= c;
  }
  return ESP_OK;
}

esp_err_t epd_draw_full(epd_handle_t *epd, const uint8_t *framebuf) {
  ESP_ERROR_CHECK(set_ram_ptr(epd, 0x00, 0x0000));
  ESP_ERROR_CHECK(cmd1(epd, CMD_WRITE_RAM_BW));
  const size_t n = (EPD_WIDTH * EPD_HEIGHT) / 8;
  // Push in modest chunks for DMA
  size_t left = n;
  const uint8_t *p = framebuf;
  while (left) {
    size_t c = left > 1024 ? 1024 : left;
    ESP_ERROR_CHECK(data(epd, p, c));
    p += c;
    left -= c;
  }
  return ESP_OK;
}

esp_err_t epd_refresh_full(epd_handle_t *epd) {
  // DISPLAY_UPDATE_CONTROL_2 bits (0x22):
  // typical full refresh: enable clock + enable analog + display mode + load
  // LUT (0xC7 is common) then MASTER_ACTIVATION (0x20), wait BUSY → idle.
  // :contentReference[oaicite:6]{index=6}
  ESP_ERROR_CHECK(cmd1(epd, CMD_DISPLAY_UPDATE_CONTROL_2));
  ESP_ERROR_CHECK(data1(epd, 0xC7));
  ESP_ERROR_CHECK(cmd1(epd, CMD_MASTER_ACTIVATION));
  wait_busy(epd);
  return ESP_OK;
}

esp_err_t epd_draw_area(epd_handle_t *epd, int x, int y, int w, int h,
                        const uint8_t *bits) {
  if (x < 0 || y < 0 || w <= 0 || h <= 0)
    return ESP_ERR_INVALID_ARG;
  if (x + w > EPD_WIDTH || y + h > EPD_HEIGHT)
    return ESP_ERR_INVALID_ARG;

  // set window (note X in bytes)
  uint8_t x0b = (uint8_t)(x / 8);
  uint8_t x1b = (uint8_t)((x + w - 1) / 8);
  ESP_ERROR_CHECK(cmd1(epd, CMD_SET_RAM_X_START_END));
  uint8_t xse[] = {x0b, x1b};
  ESP_ERROR_CHECK(data(epd, xse, sizeof(xse)));

  uint16_t y0 = (uint16_t)y, y1 = (uint16_t)(y + h - 1);
  ESP_ERROR_CHECK(cmd1(epd, CMD_SET_RAM_Y_START_END));
  uint8_t yse[] = {(uint8_t)(y0 & 0xFF), (uint8_t)(y0 >> 8),
                   (uint8_t)(y1 & 0xFF), (uint8_t)(y1 >> 8)};
  ESP_ERROR_CHECK(data(epd, yse, sizeof(yse)));

  // write rows
  for (int row = 0; row < h; ++row) {
    ESP_ERROR_CHECK(set_ram_ptr(epd, x0b, (uint16_t)(y + row)));
    ESP_ERROR_CHECK(cmd1(epd, CMD_WRITE_RAM_BW));
    size_t row_bytes = (size_t)(x1b - x0b + 1);
    ESP_ERROR_CHECK(data(epd, bits + row * row_bytes, row_bytes));
  }

  // for simplicity do a full refresh (partial needs tuned LUTs per controller)
  return epd_refresh_full(epd);
}

esp_err_t epd_sleep(epd_handle_t *epd) {
  ESP_ERROR_CHECK(cmd1(epd, CMD_DEEP_SLEEP));
  ESP_ERROR_CHECK(data1(epd, 0x01)); // deep sleep
  delay_ms(10);
  return ESP_OK;
}
