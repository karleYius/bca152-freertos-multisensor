#ifndef DHT22_H
#define DHT22_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t dht22_read(
    int gpio_num,
    float *temperature,
    float *humidity
);

#ifdef __cplusplus
}
#endif

#endif