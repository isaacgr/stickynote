#include "epd2in7.h"
#include <stdlib.h>

struct Epd {
  unsigned width;
  unsigned height;

  unsigned reset_pin;
  unsigned dc_pin;
  unsigned cs_pin;
  unsigned busy_pin;

  uint8_t *framebuf;
  size_t *framebuf_len;
};

Epd *epd_create(void) {
  Epd *e = (Epd *)calloc(1, sizeof(*e));
  if (!e) {
    return NULL;
  }
  e->width = EPD2IN7_WIDTH;
  e->height = EPD2IN7_HEIGHT;

  return e;
}
