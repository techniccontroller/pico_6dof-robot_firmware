#include "comm_functions.h"

static uint16_t _get_block(uint8_t *buffer, uint8_t len);

void comm_func_init()
{
    stdio_init_all();
}

uint16_t comm_func_read_all_bytes(uint8_t *dst, uint8_t len)
{
    return _get_block(dst, len);
}

void comm_func_write(const char *s)
{
    printf(s);
}

static uint16_t _get_block(uint8_t *buffer, uint8_t len) {
  uint16_t buffer_index= 0;
  while (true) {
    int c = getchar_timeout_us(100);
    if (c != PICO_ERROR_TIMEOUT && buffer_index < len) {
      buffer[buffer_index++] = (c & 0xFF);
    } else {
      break;
    }
  }
  return buffer_index;
}
