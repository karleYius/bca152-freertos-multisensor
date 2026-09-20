#include <stdio.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dht22.h"

#define DHT22_GPIO 4
#define LDR_CHANNEL ADC_CHANNEL_6

adc_oneshot_unit_handle_t adc_handle;

const char *TAG = "MAIN";

// Function declaration
static void sensor_task(void *pvParameters);

void app_main()
{
    ESP_LOGI(TAG, "BCA152 FreeRTOS Multisensor");
    ESP_LOGI(TAG, "System Starting...");

    // Configure ADC
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &adc_config,
            &adc_handle
        )
    );

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            LDR_CHANNEL,
            &channel_config
        )
    );

    // Start Sensor Task
    xTaskCreate(
        sensor_task,
        "Sensor Task",
        4096,
        NULL,
        5,
        NULL
    );
}

static void sensor_task(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    float temperature;
    float humidity;
    int light_raw;

    while (1)
    {
        // Read DHT22
        esp_err_t dht_result =
            dht22_read(
                DHT22_GPIO,
                &temperature,
                &humidity
            );

        // Read LDR
        esp_err_t ldr_result =
            adc_oneshot_read(
                adc_handle,
                LDR_CHANNEL,
                &light_raw
            );

        // Print DHT22 values
        if (dht_result == ESP_OK)
        {
            printf("Temperature: %.2f C\n", temperature);
            printf("Humidity: %.2f %%\n", humidity);
        }
        else
        {
            printf(
                "DHT22 read failed: %s\n",
                esp_err_to_name(dht_result)
            );
        }

        // Print LDR value
        if (ldr_result == ESP_OK)
        {
            int light_percent =
                (light_raw * 100) / 4095;

            printf(
                "Light: %d%% (ADC: %d)\n",
                light_percent,
                light_raw
            );
        }
        else
        {
            printf(
                "LDR read failed: %s\n",
                esp_err_to_name(ldr_result)
            );
        }

        printf("----------------------\n");

        // Run every 2 seconds
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}