#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

esp_err_t ssd1306_init(
    i2c_master_bus_handle_t bus_handle,
    uint8_t i2c_address
);

esp_err_t ssd1306_clear(void);

esp_err_t ssd1306_set_cursor(
    uint8_t x,
    uint8_t y
);

esp_err_t ssd1306_write_text(
    const char *text
);

esp_err_t ssd1306_update(void);

#ifdef __cplusplus
}
#endif

#endif