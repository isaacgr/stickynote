#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "utils.h"
#include <stdio.h>
#include <sys/time.h>

#define BLINK_GPIO 2

void gpio_toggle(gpio_num_t gpio_num) {
  int state = !gpio_get_level(gpio_num);
  gpio_set_level(gpio_num, state);
}

int64_t tick_us(int64_t offset) {
  static struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec + offset;
}

void delay_until(int64_t t) {
  static struct timeval tv_now;
  do {
    gettimeofday(&tv_now, NULL);
  } while (t > (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec);
}

void app_main(void) {

  gpio_reset_pin(BLINK_GPIO);
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_INPUT_OUTPUT);

  print_chip_info();

  while (1) {
    gpio_toggle(BLINK_GPIO);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
