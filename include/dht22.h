#ifndef DHT22_H
#define DHT22_H

#include "esp_err.h"

esp_err_t dht22_read(int gpio_num, float *temperature, float *humidity);

#endif