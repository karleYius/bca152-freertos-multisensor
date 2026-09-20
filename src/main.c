#include <stdio.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "dht22.h"
#include "ssd1306.h"


// ======================================================
// GPIO / HARDWARE CONFIGURATION
// ======================================================

#define DHT22_GPIO 4

// ADC1 Channel 6 = GPIO34 on ESP32
#define LDR_CHANNEL ADC_CHANNEL_6

#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22

#define OLED_I2C_ADDRESS 0x3C


// ======================================================
// SENSOR DATA STRUCTURE
// ======================================================

struct SensorData
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};


// ======================================================
// GLOBAL HANDLES
// ======================================================

adc_oneshot_unit_handle_t adc_handle;

i2c_master_bus_handle_t i2c_bus_handle;

QueueHandle_t sensor_queue;


// ======================================================
// LOG TAG
// ======================================================

static const char *TAG = "MAIN";


// ======================================================
// TASK FUNCTION DECLARATIONS
// ======================================================

static void sensor_task(void *pvParameters);

static void display_task(void *pvParameters);


// ======================================================
// APP MAIN
// ======================================================

void app_main(void)
{
    ESP_LOGI(TAG, "BCA152 FreeRTOS Multisensor");
    ESP_LOGI(TAG, "System Starting...");


    // ==================================================
    // CONFIGURE ADC FOR LDR
    // ==================================================

    adc_oneshot_unit_init_cfg_t adc_config =
    {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &adc_config,
            &adc_handle
        )
    );


    adc_oneshot_chan_cfg_t channel_config =
    {
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


    // ==================================================
    // CONFIGURE I2C BUS
    // ==================================================

    i2c_master_bus_config_t i2c_config =
    {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &i2c_config,
            &i2c_bus_handle
        )
    );


    // ==================================================
    // CREATE SENSOR QUEUE
    // ==================================================

    sensor_queue = xQueueCreate(
        10,
        sizeof(struct SensorData)
    );

    if (sensor_queue == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to create sensor queue"
        );

        return;
    }


    // ==================================================
    // CREATE SENSOR TASK
    // ==================================================

    BaseType_t sensor_task_result =
        xTaskCreate(
            sensor_task,
            "SensorTask",
            4096,
            NULL,
            5,
            NULL
        );

    if (sensor_task_result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create SensorTask"
        );

        return;
    }


    // ==================================================
    // CREATE DISPLAY TASK
    // ==================================================

    BaseType_t display_task_result =
        xTaskCreate(
            display_task,
            "DisplayTask",
            4096,
            NULL,
            4,
            NULL
        );

    if (display_task_result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create DisplayTask"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "FreeRTOS tasks created successfully"
    );
}


// ======================================================
// SENSOR TASK
// ======================================================

static void sensor_task(void *pvParameters)
{
    /*
     * vTaskDelayUntil() requires a reference time.
     *
     * This lets SensorTask run approximately every
     * 2000 ms without accumulating timing drift.
     */
    TickType_t lastWakeTime =
        xTaskGetTickCount();


    float temperature = 0.0f;
    float humidity = 0.0f;

    int light_raw = 0;


    /*
     * Initialize the structure so that it never contains
     * random/uninitialized values.
     */
    struct SensorData sensor_data =
    {
        .temperature = 0.0f,
        .humidity = 0.0f,
        .lightLevel = 0,
        .motionDetected = false
    };


    while (1)
    {
        // ==============================================
        // READ DHT22
        // ==============================================

        esp_err_t dht_result =
            dht22_read(
                DHT22_GPIO,
                &temperature,
                &humidity
            );


        if (dht_result == ESP_OK)
        {
            sensor_data.temperature =
                temperature;

            sensor_data.humidity =
                humidity;
        }
        else
        {
            ESP_LOGW(
                "SensorTask",
                "DHT22 read failed: %s",
                esp_err_to_name(dht_result)
            );
        }


        // ==============================================
        // READ LDR
        // ==============================================

        esp_err_t ldr_result =
            adc_oneshot_read(
                adc_handle,
                LDR_CHANNEL,
                &light_raw
            );


        if (ldr_result == ESP_OK)
        {
            /*
             * Convert ADC reading from approximately
             * 0-4095 into 0-100%.
             */
            sensor_data.lightLevel =
                (light_raw * 100) / 4095;
        }
        else
        {
            ESP_LOGW(
                "SensorTask",
                "LDR read failed: %s",
                esp_err_to_name(ldr_result)
            );
        }


        // ==============================================
        // MOTION SENSOR
        // ==============================================

        /*
         * PIR sensor will be implemented in a later task.
         */
        sensor_data.motionDetected =
            false;


        // ==============================================
        // SEND SENSOR DATA TO DISPLAY TASK
        // ==============================================

        if (
            xQueueSend(
                sensor_queue,
                &sensor_data,
                pdMS_TO_TICKS(100)
            ) != pdPASS
        )
        {
            ESP_LOGW(
                "SensorTask",
                "Sensor queue is full"
            );
        }


        // ==============================================
        // SERIAL OUTPUT FOR TESTING
        // ==============================================

        printf(
            "\n-----------------------------\n"
        );

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
            sensor_data.motionDetected
                ? "YES"
                : "NO"
        );

        printf(
            "-----------------------------\n"
        );


        // ==============================================
        // RUN SENSOR TASK EVERY 2 SECONDS
        // ==============================================

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}


// ======================================================
// DISPLAY TASK
// ======================================================

static void display_task(void *pvParameters)
{
    struct SensorData received_data;

    char temperature_text[32];


    // ==================================================
    // INITIALIZE OLED INSIDE DISPLAY TASK
    // ==================================================

    /*
     * DisplayTask owns the OLED.
     *
     * This means other tasks should NOT call
     * ssd1306_clear(), ssd1306_write_text(),
     * ssd1306_update(), etc.
     */

    esp_err_t oled_result =
        ssd1306_init(
            i2c_bus_handle,
            OLED_I2C_ADDRESS
        );


    if (oled_result != ESP_OK)
    {
        ESP_LOGE(
            "DisplayTask",
            "OLED initialization failed: %s",
            esp_err_to_name(oled_result)
        );

        /*
         * Delete this task because the display
         * cannot be used.
         */
        vTaskDelete(NULL);
    }


    ESP_LOGI(
        "DisplayTask",
        "OLED initialized successfully"
    );


    // ==================================================
    // INITIAL DISPLAY
    // ==================================================

    ESP_ERROR_CHECK(
        ssd1306_clear()
    );

    ESP_ERROR_CHECK(
        ssd1306_set_cursor(
            0,
            0
        )
    );

    ESP_ERROR_CHECK(
        ssd1306_write_text(
            "ROOM MONITOR"
        )
    );


    ESP_ERROR_CHECK(
        ssd1306_set_cursor(
            0,
            2
        )
    );

    ESP_ERROR_CHECK(
        ssd1306_write_text(
            "WAITING..."
        )
    );


    ESP_ERROR_CHECK(
        ssd1306_update()
    );


    // ==================================================
    // DISPLAY LOOP
    // ==================================================

    while (1)
    {
        /*
         * Wait here until SensorTask sends something.
         *
         * portMAX_DELAY means DisplayTask blocks instead
         * of constantly using the CPU.
         */
        if (
            xQueueReceive(
                sensor_queue,
                &received_data,
                portMAX_DELAY
            ) == pdTRUE
        )
        {
            // ==========================================
            // CREATE TEMPERATURE STRING
            // ==========================================

            snprintf(
                temperature_text,
                sizeof(temperature_text),
                "%.1f C",
                received_data.temperature
            );


            // ==========================================
            // CLEAR PREVIOUS OLED CONTENT
            // ==========================================

            ESP_ERROR_CHECK(
                ssd1306_clear()
            );


            // ==========================================
            // LINE 1
            // ==========================================

            ESP_ERROR_CHECK(
                ssd1306_set_cursor(
                    0,
                    0
                )
            );

            ESP_ERROR_CHECK(
                ssd1306_write_text(
                    "ROOM MONITOR"
                )
            );


            // ==========================================
            // LINE 2
            // ==========================================

            ESP_ERROR_CHECK(
                ssd1306_set_cursor(
                    0,
                    2
                )
            );

            ESP_ERROR_CHECK(
                ssd1306_write_text(
                    "TEMPERATURE"
                )
            );


            // ==========================================
            // LINE 3
            // ==========================================

            ESP_ERROR_CHECK(
                ssd1306_set_cursor(
                    0,
                    4
                )
            );

            ESP_ERROR_CHECK(
                ssd1306_write_text(
                    temperature_text
                )
            );


            // ==========================================
            // SEND BUFFER TO OLED
            // ==========================================

            ESP_ERROR_CHECK(
                ssd1306_update()
            );


            ESP_LOGI(
                "DisplayTask",
                "OLED updated: %.1f C",
                received_data.temperature
            );
        }
    }
}