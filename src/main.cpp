#include <stdio.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "dht22.h"
#include "ssd1306.h"


// ======================================================
// HARDWARE CONFIGURATION
// ======================================================

// DHT22
#define DHT22_GPIO 4

// LDR
// ADC1 Channel 6 = GPIO34 on classic ESP32
#define LDR_CHANNEL ADC_CHANNEL_6

// OLED
#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22
#define OLED_I2C_ADDRESS 0x3C

// Rotary Encoder
#define ENCODER_CLK_GPIO 18
#define ENCODER_DT_GPIO  19
#define ENCODER_SW_GPIO  23


// ======================================================
// DISPLAY MODES
// ======================================================

typedef enum
{
    DISPLAY_TEMPERATURE,
    DISPLAY_HUMIDITY,
    DISPLAY_LIGHT,
    DISPLAY_MOTION

} DisplayMode;


// ======================================================
// SENSOR DATA
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

SemaphoreHandle_t display_mode_mutex;


// ======================================================
// CURRENT DISPLAY MODE
// ======================================================

static DisplayMode current_display_mode =
    DISPLAY_TEMPERATURE;


// ======================================================
// LOG TAG
// ======================================================

static const char *TAG = "MAIN";


// ======================================================
// TASK DECLARATIONS
// ======================================================

static void sensor_task(void *pvParameters);

static void display_task(void *pvParameters);

static void input_task(void *pvParameters);


// ======================================================
// DISPLAY MODE HELPER
// ======================================================

static DisplayMode get_display_mode(void)
{
    DisplayMode mode;

    /*
     * Protect access to current_display_mode.
     *
     * InputTask changes it.
     * DisplayTask reads it.
     */
    if (
        xSemaphoreTake(
            display_mode_mutex,
            portMAX_DELAY
        ) == pdTRUE
    )
    {
        mode = current_display_mode;

        xSemaphoreGive(
            display_mode_mutex
        );
    }
    else
    {
        mode = DISPLAY_TEMPERATURE;
    }

    return mode;
}


// ======================================================
// NEXT DISPLAY MODE
// ======================================================

static DisplayMode next_display_mode(
    DisplayMode mode
)
{
    switch (mode)
    {
        case DISPLAY_TEMPERATURE:
            return DISPLAY_HUMIDITY;

        case DISPLAY_HUMIDITY:
            return DISPLAY_LIGHT;

        case DISPLAY_LIGHT:
            return DISPLAY_MOTION;

        case DISPLAY_MOTION:
        default:
            return DISPLAY_TEMPERATURE;
    }
}


// ======================================================
// PREVIOUS DISPLAY MODE
// ======================================================

static DisplayMode previous_display_mode(
    DisplayMode mode
)
{
    switch (mode)
    {
        case DISPLAY_TEMPERATURE:
            return DISPLAY_MOTION;

        case DISPLAY_HUMIDITY:
            return DISPLAY_TEMPERATURE;

        case DISPLAY_LIGHT:
            return DISPLAY_HUMIDITY;

        case DISPLAY_MOTION:
        default:
            return DISPLAY_LIGHT;
    }
}


// ======================================================
// SET DISPLAY MODE
// ======================================================

static void set_display_mode(
    DisplayMode mode
)
{
    if (
        xSemaphoreTake(
            display_mode_mutex,
            portMAX_DELAY
        ) == pdTRUE
    )
    {
        current_display_mode = mode;

        xSemaphoreGive(
            display_mode_mutex
        );
    }
}


// ======================================================
// APP MAIN
// ======================================================

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "BCA152 FreeRTOS Multisensor"
    );

    ESP_LOGI(
        TAG,
        "System Starting..."
    );


    // ==================================================
    // CONFIGURE ADC
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
    // CONFIGURE I2C
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
    // CREATE DISPLAY MODE MUTEX
    // ==================================================

    display_mode_mutex = xSemaphoreCreateMutex();

    if (display_mode_mutex == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to create display mode mutex"
        );

        return;
    }


    // ==================================================
    // CONFIGURE ROTARY ENCODER GPIO
    // ==================================================

    gpio_config_t encoder_config =
    {
        .pin_bit_mask =
            (1ULL << ENCODER_CLK_GPIO)
            |
            (1ULL << ENCODER_DT_GPIO)
            |
            (1ULL << ENCODER_SW_GPIO),

        .mode = GPIO_MODE_INPUT,

        .pull_up_en = GPIO_PULLUP_ENABLE,

        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(
        gpio_config(
            &encoder_config
        )
    );


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


    // ==================================================
    // CREATE INPUT TASK
    // ==================================================

    BaseType_t input_task_result =
        xTaskCreate(
            input_task,
            "InputTask",
            4096,
            NULL,
            4,
            NULL
        );

    if (input_task_result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create InputTask"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "All FreeRTOS tasks created successfully"
    );
}


// ======================================================
// SENSOR TASK
// ======================================================

static void sensor_task(void *pvParameters)
{
    TickType_t lastWakeTime =
        xTaskGetTickCount();


    float temperature = 0.0f;

    float humidity = 0.0f;

    int light_raw = 0;


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
        // MOTION
        // ==============================================

        /*
         * PIR will be implemented later.
         */
        sensor_data.motionDetected = false;


        // ==============================================
        // SEND DATA TO QUEUE
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
        // SERIAL OUTPUT
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
        // PERIODIC EXECUTION
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

    char value_text[32];


    // ==================================================
    // INITIALIZE OLED
    // ==================================================

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

        vTaskDelete(NULL);
    }


    ESP_LOGI(
        "DisplayTask",
        "OLED initialized successfully"
    );


    // ==================================================
    // INITIAL OLED SCREEN
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
            "TEMPERATURE"
        )
    );

    ESP_ERROR_CHECK(
        ssd1306_set_cursor(
            0,
            4
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
         * Wait for new sensor data.
         */
        if (
            xQueueReceive(
                sensor_queue,
                &received_data,
                portMAX_DELAY
            ) == pdTRUE
        )
        {
            DisplayMode mode =
                get_display_mode();


            // ==========================================
            // CLEAR DISPLAY
            // ==========================================

            ESP_ERROR_CHECK(
                ssd1306_clear()
            );


            // ==========================================
            // TITLE
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
            // DISPLAY SELECTED PAGE
            // ==========================================

            switch (mode)
            {
                // --------------------------------------
                // TEMPERATURE
                // --------------------------------------

                case DISPLAY_TEMPERATURE:

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

                    snprintf(
                        value_text,
                        sizeof(value_text),
                        "%.1f C",
                        received_data.temperature
                    );

                    ESP_ERROR_CHECK(
                        ssd1306_set_cursor(
                            0,
                            4
                        )
                    );

                    ESP_ERROR_CHECK(
                        ssd1306_write_text(
                            value_text
                        )
                    );

                    break;


                // --------------------------------------
                // HUMIDITY
                // --------------------------------------

                case DISPLAY_HUMIDITY:

                    ESP_ERROR_CHECK(
                        ssd1306_set_cursor(
                            0,
                            2
                        )
                    );

                    ESP_ERROR_CHECK(
                        ssd1306_write_text(
                            "HUMIDITY"
                        )
                    );

                    snprintf(
                        value_text,
                        sizeof(value_text),
                        "%.1f %%",
                        received_data.humidity
                    );

                    ESP_ERROR_CHECK(
                        ssd1306_set_cursor(
                            0,
                            4
                        )
                    );

                    ESP_ERROR_CHECK(
                        ssd1306_write_text(
                            value_text
                        )
                    );

                    break;


                // --------------------------------------
                // LIGHT
                // --------------------------------------

                case DISPLAY_LIGHT:

                    ESP_ERROR_CHECK(
                        ssd1306_set_cursor(
                            0,
                            2
                        )
                    );

                    ESP_ERROR_CHECK(
                        ssd1306_write_text(
                            "LIGHT"
                        )
                    );

                    snprintf(
                        value_text,
                        sizeof(value_text),
                        "%d %%",
                        received_data.lightLevel
                    );

                    ESP_ERROR_CHECK(
                        ssd1306_set_cursor(
                            0,
                            4
                        )
                    );

                    ESP_ERROR_CHECK(
                        ssd1306_write_text(
                            value_text
                        )
                    );

                    break;


                // --------------------------------------
                // MOTION
                // --------------------------------------

                case DISPLAY_MOTION:

                    ESP_ERROR_CHECK(
                        ssd1306_set_cursor(
                            0,
                            2
                        )
                    );

                    ESP_ERROR_CHECK(
                        ssd1306_write_text(
                            "MOTION"
                        )
                    );

                    if (
                        received_data.motionDetected
                    )
                    {
                        ESP_ERROR_CHECK(
                            ssd1306_set_cursor(
                                0,
                                4
                            )
                        );

                        ESP_ERROR_CHECK(
                            ssd1306_write_text(
                                "DETECTED"
                            )
                        );
                    }
                    else
                    {
                        ESP_ERROR_CHECK(
                            ssd1306_set_cursor(
                                0,
                                4
                            )
                        );

                        ESP_ERROR_CHECK(
                            ssd1306_write_text(
                                "NO MOTION"
                            )
                        );
                    }

                    break;


                default:

                    break;
            }


            // ==========================================
            // UPDATE OLED
            // ==========================================

            ESP_ERROR_CHECK(
                ssd1306_update()
            );
        }
    }
}


// ======================================================
// INPUT TASK
// ======================================================

static void input_task(void *pvParameters)
{
    int previous_clk =
        gpio_get_level(
            ENCODER_CLK_GPIO
        );


    while (1)
    {
        int current_clk =
            gpio_get_level(
                ENCODER_CLK_GPIO
            );


        /*
         * Detect a falling edge on CLK.
         *
         * This corresponds to one encoder movement
         * when using the standard Wokwi rotary encoder.
         */
        if (
            current_clk != previous_clk
            &&
            current_clk == 0
        )
        {
            int dt_level =
                gpio_get_level(
                    ENCODER_DT_GPIO
                );


            DisplayMode current_mode =
                get_display_mode();


            if (dt_level != current_clk)
            {
                // ======================================
                // CLOCKWISE
                // ======================================

                DisplayMode new_mode =
                    next_display_mode(
                        current_mode
                    );

                set_display_mode(
                    new_mode
                );

                ESP_LOGI(
                    "InputTask",
                    "Encoder clockwise -> mode %d",
                    new_mode
                );
            }
            else
            {
                // ======================================
                // COUNTERCLOCKWISE
                // ======================================

                DisplayMode new_mode =
                    previous_display_mode(
                        current_mode
                    );

                set_display_mode(
                    new_mode
                );

                ESP_LOGI(
                    "InputTask",
                    "Encoder counterclockwise -> mode %d",
                    new_mode
                );
            }


            /*
             * Small debounce delay.
             */
            vTaskDelay(
                pdMS_TO_TICKS(50)
            );
        }


        previous_clk = current_clk;


        /*
         * Poll encoder approximately every 5 ms.
         */
        vTaskDelay(
            pdMS_TO_TICKS(5)
        );
    }
}