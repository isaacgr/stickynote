#ifndef EPD2IN7_H
#define EPD2IN7_H

#include "driver/spi_master.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Panel geometry (landscape framebuffer layout: 264x176)
#define EPD2IN7_WIDTH 264
#define EPD2IN7_HEIGHT 176
#define EPD2IN7_FB_BYTES ((EPD2IN7_WIDTH * EPD2IN7_HEIGHT) / 8)

// ==== Common command set for EK79652 / IL91874 / SSD1680-family ====
#define PANEL_SETTING 0x00
#define POWER_SETTING 0x01
#define POWER_OFF 0x02
#define POWER_OFF_SEQUENCE_SETTING 0x03
#define POWER_ON 0x04
#define POWER_ON_MEASURE 0x05
#define BOOSTER_SOFT_START 0x06
#define DEEP_SLEEP 0x07
#define DATA_START_TRANSMISSION_1 0x10
#define DATA_STOP 0x11
#define DISPLAY_REFRESH 0x12
#define DATA_START_TRANSMISSION_2 0x13
#define PARTIAL_DATA_START_TRANSMISSION_1 0x14
#define PARTIAL_DATA_START_TRANSMISSION_2 0x15
#define PARTIAL_DISPLAY_REFRESH 0x16
#define LUT_FOR_VCOM 0x20
#define LUT_WHITE_TO_WHITE 0x21
#define LUT_BLACK_TO_WHITE 0x22
#define LUT_WHITE_TO_BLACK 0x23
#define LUT_BLACK_TO_BLACK 0x24
#define PLL_CONTROL 0x30
#define TEMPERATURE_SENSOR_COMMAND 0x40
#define TEMPERATURE_SENSOR_CALIBRATION 0x41
#define TEMPERATURE_SENSOR_WRITE 0x42
#define TEMPERATURE_SENSOR_READ 0x43
#define VCOM_AND_DATA_INTERVAL_SETTING 0x50
#define LOW_POWER_DETECTION 0x51
#define TCON_SETTING 0x60
#define TCON_RESOLUTION 0x61
#define SOURCE_AND_GATE_START_SETTING 0x62
#define GET_STATUS 0x71
#define AUTO_MEASURE_VCOM 0x80
#define VCOM_VALUE 0x81
#define VCM_DC_SETTING_REGISTER 0x82
#define PROGRAM_MODE 0xA0
#define ACTIVE_PROGRAM 0xA1
#define READ_OTP_DATA 0xA2

typedef struct Epd Epd;

/* Constructor / destructor */
Epd *epd_create(void);
void epd_destroy(Epd *epd);

/* Properties */
unsigned epd_width(const Epd *epd);
unsigned epd_height(const Epd *epd);

int epd_init(Epd *epd);
void epd_send_command(Epd *epd, uint8_t cmd);
void epd_send_data(Epd *epd, uint8_t data);
void epd_wait_until_idle(Epd *epd);
void epd_reset(Epd *epd);
void epd_set_lut(Epd *epd);
void epd_transmit_partial_data(Epd *epd, const uint8_t *buffer, int x, int y,
                               int w, int l);
void epd_refresh_partial(Epd *epd, int x, int y, int w, int l);
void epd_display_frame_buf(Epd *epd, const uint8_t *frame_buffer);
void epd_display_frame(Epd *epd); /* use internal buffer if you keep one */
void epd_clear_frame(Epd *epd);
void epd_sleep(Epd *epd);

/* 4-gray variants */
void epd_init_4gray(Epd *epd);
void epd_gray_set_lut(Epd *epd);
void epd_display_4gray(Epd *epd, const uint8_t *image);

#endif
