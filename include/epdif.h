#ifndef EPDIF_H
#define EPDIF_H

#include "driver/spi_master.h"
#include "esp_idf_version.h"
#include <stdint.h>

#if ESP_IDF_VERSION_MAJOR >= 4
#ifndef SPI_HOST
#define SPI_HOST SPI2_HOST /* AKA HSPI on older chips */
#endif
#endif

int EpdIf_IfInit(void);
void EpdIf_DigitalWrite(uint32_t pin, int value);
int EpdIf_DigitalRead(uint32_t pin);
void EpdIf_DelayMs(uint32_t ms);
void EpdIf_SpiTransfer(uint8_t data);

#endif
