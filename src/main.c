#include <stdio.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "dht22.h"

#define DHT22_GPIO 4
#define LDR_CHANNEL ADC_CHANNEL_6

struct SensorData {
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};

adc_oneshot_unit_handle_t adc_handle;
QueueHandle_t sensor_queue;

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

    // Create Sensor Queue
    sensor_queue = xQueueCreate(
        10,
        sizeof(struct SensorData)
    );

    if (sensor_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sensor queue");
        return;
    }

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

    struct SensorData sensor_data;

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

        // Store DHT22 data
        if (dht_result == ESP_OK)
        {
            sensor_data.temperature = temperature;
            sensor_data.humidity = humidity;
        }
        else
        {
            printf(
                "DHT22 read failed: %s\n",
                esp_err_to_name(dht_result)
            );
        }

        // Store LDR data
        if (ldr_result == ESP_OK)
        {
            sensor_data.lightLevel =
                (light_raw * 100) / 4095;
        }
        else
        {
            printf(
                "LDR read failed: %s\n",
                esp_err_to_name(ldr_result)
            );
        }

        // Motion sensor will be implemented later
        sensor_data.motionDetected = false;

        // Send data to queue
        if (xQueueSend(
                sensor_queue,
                &sensor_data,
                0
            ) != pdPASS)
        {
            printf("Failed to send sensor data to queue\n");
        }

        // Print values for testing
        printf(
            "Temperature: %.2f C\n",
            sensor_data.temperature
        );

        printf(
            "Humidity: %.2f %%\n",
            sensor_data.humidity
        );

        printf(
            "Light: %d%%\n",
            sensor_data.lightLevel
        );

        printf(
            "Motion: %s\n",
            sensor_data.motionDetected ? "YES" : "NO"
        );

        printf("----------------------\n");

        // Run every 2 seconds
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}