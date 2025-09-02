#include "epdif.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static spi_device_handle_t s_epd_spi = NULL;

// Helper functions to configure gpio
static inline void cfg_output(gpio_num_t pin)
{
    gpio_config_t io = { .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&io);
}

static inline void cfg_input(gpio_num_t pin)
{
    gpio_config_t io = { .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&io);
}

int EpdIf_IfInit(void)
{

    cfg_output((gpio_num_t)EPD_PIN_RST);
    cfg_output((gpio_num_t)EPD_PIN_DC);
    cfg_output((gpio_num_t)EPD_PIN_CS);
    cfg_input((gpio_num_t)EPD_PIN_BUSY);

    spi_bus_config_t buscfg = { .mosi_io_num = EPD_PIN_MOSI,
        .miso_io_num = -1, // epd is write only
        .sclk_io_num = EPD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096 };

    esp_err_t err = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return -1;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10MHZ
        .mode = 0, // CPOL=0, CPHA=0 typical
        .spics_io_num = EPD_PIN_CS, // use HW-controlled CS
        .queue_size = 4,
        .flags = 0
    };

    // Attach the EPD to the SPI bus, skip if already added
    if (!s_epd_spi) {
        err = spi_bus_add_device(SPI_HOST, &devcfg, &s_epd_spi);
        if (err != ESP_OK) {
            return -1;
        }
    }
    return 0;
}

void EpdIf_DigitalWrite(uint32_t pin, int value)
{
    gpio_set_level((gpio_num_t)pin, value ? 1 : 0);
}

int EpdIf_DigitalRead(uint32_t pin) { return gpio_get_level((gpio_num_t)pin); }

void EpdIf_DelayMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

void EpdIf_SpiTransfer(uint8_t data)
{
    if (!s_epd_spi)
        return;
    spi_transaction_t t = { 0 };
    t.length = 8;
    t.tx_buffer = &data;
    (void)spi_device_transmit(s_epd_spi, &t);
}
