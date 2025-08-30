
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_private/esp_clk.h"

static const char *chip_model_name(esp_chip_model_t m) {
  switch (m) {
  case CHIP_ESP32:
    return "ESP32";
  case CHIP_ESP32S2:
    return "ESP32-S2";
  case CHIP_ESP32S3:
    return "ESP32-S3";
  case CHIP_ESP32C2:
    return "ESP32-C2";
  case CHIP_ESP32C3:
    return "ESP32-C3";
  case CHIP_ESP32C6:
    return "ESP32-C6";
  case CHIP_ESP32H2:
    return "ESP32-H2";
  default:
    return "Unknown";
  }
}

void print_chip_info() {

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  printf("Chip model: %s\n", chip_model_name(chip_info.model));
  printf("Cores: %d\n", chip_info.cores);
  printf("Revision: %d\n", chip_info.revision);
  printf("CPU Freq (MHz): %ld\n", (long)(esp_clk_cpu_freq() / 1000000));

  // Heap information
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
  size_t min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);

  printf("Total heap: %u bytes\n", (unsigned)total_heap);
  printf("Free heap: %u bytes\n", (unsigned)free_heap);
  printf("Min free heap ever: %u bytes\n", (unsigned)min_free_heap);
  // Flash size (IDF v5 API)
  uint32_t flash_bytes = 0;
  esp_err_t err = esp_flash_get_size(NULL, &flash_bytes); // NULL = default chip
  if (err == ESP_OK) {
    printf("Flash Size: %u MB\n", (unsigned)(flash_bytes / (1024 * 1024)));
  } else {
    printf("Flash Size: <error %d>\n", (int)err);
  }

  // Unique ID / MAC address (first MAC is WiFi STA)
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  printf("MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2],
         mac[3], mac[4], mac[5]);
  printf("here");
}
