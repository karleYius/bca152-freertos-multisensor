#include "dht22.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define DHT_TIMEOUT_US 1000

static int wait_for_level(int gpio_num, int level, int timeout_us)
{
    int64_t start = esp_timer_get_time();

    while (gpio_get_level(gpio_num) != level)
    {
        if ((esp_timer_get_time() - start) > timeout_us)
        {
            return -1;
        }
    }

    return (int)(esp_timer_get_time() - start);
}

esp_err_t dht22_read(int gpio_num, float *temperature, float *humidity)
{
    uint8_t data[5] = {0};

    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num, 0);

    // Start signal
    esp_rom_delay_us(2000);

    gpio_set_level(gpio_num, 1);
    esp_rom_delay_us(30);

    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);

    // Wait for DHT22 response
    if (wait_for_level(gpio_num, 0, DHT_TIMEOUT_US) < 0)
        return ESP_ERR_TIMEOUT;

    if (wait_for_level(gpio_num, 1, DHT_TIMEOUT_US) < 0)
        return ESP_ERR_TIMEOUT;

    if (wait_for_level(gpio_num, 0, DHT_TIMEOUT_US) < 0)
        return ESP_ERR_TIMEOUT;

    // Read 40 bits
    for (int i = 0; i < 40; i++)
    {
        if (wait_for_level(gpio_num, 1, DHT_TIMEOUT_US) < 0)
            return ESP_ERR_TIMEOUT;

        int high_time = wait_for_level(gpio_num, 0, DHT_TIMEOUT_US);

        if (high_time < 0)
            return ESP_ERR_TIMEOUT;

        data[i / 8] <<= 1;

        if (high_time > 40)
            data[i / 8] |= 1;
    }

    // Check checksum
    uint8_t checksum =
        data[0] + data[1] + data[2] + data[3];

    if (checksum != data[4])
    {
        return ESP_ERR_INVALID_CRC;
    }

    // Humidity
    uint16_t raw_humidity =
        ((uint16_t)data[0] << 8) | data[1];

    *humidity = raw_humidity / 10.0f;

    // Temperature
    uint16_t raw_temperature =
        ((uint16_t)(data[2] & 0x7F) << 8) | data[3];

    *temperature = raw_temperature / 10.0f;

    if (data[2] & 0x80)
    {
        *temperature = -*temperature;
    }

    return ESP_OK;
}