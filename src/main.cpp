#include <stdio.h>

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "dht22.h"
#include "ssd1306.h"
#include "alarm_logic.h"


/* =========================================================
 * GPIO / HARDWARE CONFIGURATION
 * ========================================================= */

#define DHT22_GPIO              GPIO_NUM_4

#define LDR_CHANNEL             ADC_CHANNEL_6

#define I2C_SDA_GPIO            GPIO_NUM_21
#define I2C_SCL_GPIO            GPIO_NUM_22
#define OLED_I2C_ADDRESS        0x3C

/* Rotary Encoder */
#define ENCODER_CLK_GPIO        GPIO_NUM_18
#define ENCODER_DT_GPIO         GPIO_NUM_19
#define ENCODER_SW_GPIO         GPIO_NUM_23


/* =========================================================
 * DISPLAY MODES
 * ========================================================= */

enum class DisplayMode
{
    TEMPERATURE,
    HUMIDITY,
    LIGHT,
    MOTION
};


/* =========================================================
 * SENSOR DATA
 * ========================================================= */

struct SensorData
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};


/* =========================================================
 * GLOBAL VARIABLES
 * ========================================================= */

static const char *TAG = "MAIN";

static adc_oneshot_unit_handle_t adc_handle;

static QueueHandle_t sensor_queue;

static i2c_master_bus_handle_t i2c_bus_handle;

static DisplayMode currentMode =
    DisplayMode::TEMPERATURE;


/* =========================================================
 * FUNCTION DECLARATIONS
 * ========================================================= */

static void sensor_task(void *pvParameters);

static void input_task(void *pvParameters);

static void display_task(void *pvParameters);

static const char *displayModeName(DisplayMode mode);


/* =========================================================
 * APP MAIN
 * ========================================================= */

extern "C" void app_main()
{
    ESP_LOGI(
        TAG,
        "BCA152 FreeRTOS Multisensor"
    );

    ESP_LOGI(
        TAG,
        "System Starting..."
    );


    /* =====================================================
     * CONFIGURE ADC
     * ===================================================== */

    adc_oneshot_unit_init_cfg_t adc_config = {};

    adc_config.unit_id = ADC_UNIT_1;


    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &adc_config,
            &adc_handle
        )
    );


    adc_oneshot_chan_cfg_t channel_config = {};

    channel_config.atten = ADC_ATTEN_DB_12;
    channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;


    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            LDR_CHANNEL,
            &channel_config
        )
    );


    ESP_LOGI(
        TAG,
        "ADC configured"
    );


    /* =====================================================
     * CONFIGURE I2C BUS
     * ===================================================== */

    i2c_master_bus_config_t i2c_config = {};

    i2c_config.i2c_port = I2C_NUM_0;
    i2c_config.sda_io_num = I2C_SDA_GPIO;
    i2c_config.scl_io_num = I2C_SCL_GPIO;
    i2c_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_config.glitch_ignore_cnt = 7;

    i2c_config.flags.enable_internal_pullup = true;


    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &i2c_config,
            &i2c_bus_handle
        )
    );


    ESP_LOGI(
        TAG,
        "I2C bus configured"
    );


    /* =====================================================
     * INITIALIZE OLED
     * ===================================================== */

    ESP_ERROR_CHECK(
        ssd1306_init(
            i2c_bus_handle,
            OLED_I2C_ADDRESS
        )
    );


    ESP_LOGI(
        TAG,
        "OLED initialized"
    );


    /* =====================================================
     * CONFIGURE ROTARY ENCODER
     * ===================================================== */

    gpio_config_t encoder_config = {};

    encoder_config.pin_bit_mask =
        (1ULL << ENCODER_CLK_GPIO) |
        (1ULL << ENCODER_DT_GPIO) |
        (1ULL << ENCODER_SW_GPIO);

    encoder_config.mode = GPIO_MODE_INPUT;

    encoder_config.pull_up_en = GPIO_PULLUP_ENABLE;

    encoder_config.pull_down_en =
        GPIO_PULLDOWN_DISABLE;

    encoder_config.intr_type =
        GPIO_INTR_DISABLE;


    ESP_ERROR_CHECK(
        gpio_config(&encoder_config)
    );


    ESP_LOGI(
        TAG,
        "Rotary encoder configured"
    );


    /* =====================================================
     * CREATE SENSOR QUEUE
     * ===================================================== */

    sensor_queue = xQueueCreate(
        10,
        sizeof(SensorData)
    );


    if (sensor_queue == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to create sensor queue"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "Sensor queue created"
    );


    /* =====================================================
     * START SENSOR TASK
     * ===================================================== */

    BaseType_t sensor_result =
        xTaskCreate(
            sensor_task,
            "SensorTask",
            4096,
            NULL,
            5,
            NULL
        );


    if (sensor_result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create SensorTask"
        );

        return;
    }


    /* =====================================================
     * START INPUT TASK
     * ===================================================== */

    BaseType_t input_result =
        xTaskCreate(
            input_task,
            "InputTask",
            4096,
            NULL,
            5,
            NULL
        );


    if (input_result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create InputTask"
        );

        return;
    }


    /* =====================================================
     * START DISPLAY TASK
     * ===================================================== */

    BaseType_t display_result =
        xTaskCreate(
            display_task,
            "DisplayTask",
            4096,
            NULL,
            4,
            NULL
        );


    if (display_result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create DisplayTask"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "All tasks started"
    );
}


/* =========================================================
 * SENSOR TASK
 * ========================================================= */

static void sensor_task(void *pvParameters)
{
    TickType_t lastWakeTime =
        xTaskGetTickCount();


    float temperature = 0.0f;

    float humidity = 0.0f;

    int light_raw = 0;


    SensorData sensor_data = {};

    sensor_data.temperature = 0.0f;
    sensor_data.humidity = 0.0f;
    sensor_data.lightLevel = 0;
    sensor_data.motionDetected = false;


    while (1)
    {
        /* =================================================
         * READ DHT22
         * ================================================= */

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
            printf(
                "DHT22 read failed: %s\n",
                esp_err_to_name(dht_result)
            );
        }


        /* =================================================
         * READ LDR
         * ================================================= */

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
            printf(
                "LDR read failed: %s\n",
                esp_err_to_name(ldr_result)
            );
        }


        /* =================================================
         * MOTION
         *
         * Keep current implementation.
         * ================================================= */

        sensor_data.motionDetected = false;


        /* =================================================
         * TEMPERATURE ALARM DECISION
         * ================================================= */

        AlarmState alarmState =
            evaluateTemperature(
                sensor_data.temperature
            );


        switch (alarmState)
        {
            case AlarmState::NORMAL:

                printf(
                    "Alarm: NORMAL\n"
                );

                break;


            case AlarmState::LOW_TEMPERATURE:

                printf(
                    "Alarm: LOW TEMPERATURE\n"
                );

                break;


            case AlarmState::HIGH_TEMPERATURE:

                printf(
                    "Alarm: HIGH TEMPERATURE\n"
                );

                break;
        }


        /* =================================================
         * SEND DATA TO QUEUE
         * ================================================= */

        if (
            xQueueSend(
                sensor_queue,
                &sensor_data,
                0
            ) != pdPASS
        )
        {
            printf(
                "Failed to send sensor data to queue\n"
            );
        }


        /* =================================================
         * PRINT SENSOR VALUES
         * ================================================= */

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
            "----------------------\n"
        );


        /* =================================================
         * WAIT 2 SECONDS
         * ================================================= */

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}


/* =========================================================
 * INPUT TASK
 * ========================================================= */

static void input_task(void *pvParameters)
{
    int previousCLK =
        gpio_get_level(
            ENCODER_CLK_GPIO
        );


    TickType_t lastRotationTime = 0;


    while (1)
    {
        int currentCLK =
            gpio_get_level(
                ENCODER_CLK_GPIO
            );


        /* =================================================
         * DETECT FALLING EDGE
         * ================================================= */

        if (
            previousCLK == 1 &&
            currentCLK == 0
        )
        {
            TickType_t now =
                xTaskGetTickCount();


            /* =============================================
             * DEBOUNCE
             * ============================================= */

            if (
                (now - lastRotationTime) >=
                pdMS_TO_TICKS(50)
            )
            {
                int dt =
                    gpio_get_level(
                        ENCODER_DT_GPIO
                    );


                /* =========================================
                 * CLOCKWISE
                 * ========================================= */

                if (dt == 1)
                {
                    switch (currentMode)
                    {
                        case DisplayMode::TEMPERATURE:

                            currentMode =
                                DisplayMode::HUMIDITY;

                            break;


                        case DisplayMode::HUMIDITY:

                            currentMode =
                                DisplayMode::LIGHT;

                            break;


                        case DisplayMode::LIGHT:

                            currentMode =
                                DisplayMode::MOTION;

                            break;


                        case DisplayMode::MOTION:

                            currentMode =
                                DisplayMode::TEMPERATURE;

                            break;
                    }


                    printf(
                        "Encoder: CLOCKWISE -> %s\n",
                        displayModeName(
                            currentMode
                        )
                    );
                }


                /* =========================================
                 * COUNTERCLOCKWISE
                 * ========================================= */

                else
                {
                    switch (currentMode)
                    {
                        case DisplayMode::TEMPERATURE:

                            currentMode =
                                DisplayMode::MOTION;

                            break;


                        case DisplayMode::HUMIDITY:

                            currentMode =
                                DisplayMode::TEMPERATURE;

                            break;


                        case DisplayMode::LIGHT:

                            currentMode =
                                DisplayMode::HUMIDITY;

                            break;


                        case DisplayMode::MOTION:

                            currentMode =
                                DisplayMode::LIGHT;

                            break;
                    }


                    printf(
                        "Encoder: COUNTERCLOCKWISE -> %s\n",
                        displayModeName(
                            currentMode
                        )
                    );
                }


                lastRotationTime = now;
            }
        }


        previousCLK = currentCLK;


        vTaskDelay(
            pdMS_TO_TICKS(5)
        );
    }
}


/* =========================================================
 * DISPLAY TASK
 * ========================================================= */

static void display_task(void *pvParameters)
{
    SensorData sensor_data = {};

    sensor_data.temperature = 0.0f;
    sensor_data.humidity = 0.0f;
    sensor_data.lightLevel = 0;
    sensor_data.motionDetected = false;


    while (1)
    {
        if (
            xQueueReceive(
                sensor_queue,
                &sensor_data,
                pdMS_TO_TICKS(2500)
            ) == pdPASS
        )
        {
            char line2[32];
            char line3[32];


            /* =============================================
             * CLEAR OLED
             * ============================================= */

            ssd1306_clear();


            /* =============================================
             * HEADER
             * ============================================= */

            ssd1306_set_cursor(
                0,
                0
            );

            ssd1306_write_text(
                "ROOM MONITOR"
            );


            /* =============================================
             * DISPLAY CURRENT PAGE
             * ============================================= */

            switch (currentMode)
            {
                case DisplayMode::TEMPERATURE:

                    snprintf(
                        line2,
                        sizeof(line2),
                        "TEMPERATURE"
                    );


                    snprintf(
                        line3,
                        sizeof(line3),
                        "%.1f C",
                        sensor_data.temperature
                    );


                    ssd1306_set_cursor(
                        0,
                        2
                    );

                    ssd1306_write_text(
                        line2
                    );


                    ssd1306_set_cursor(
                        0,
                        4
                    );

                    ssd1306_write_text(
                        line3
                    );

                    break;


                case DisplayMode::HUMIDITY:

                    snprintf(
                        line2,
                        sizeof(line2),
                        "HUMIDITY"
                    );


                    snprintf(
                        line3,
                        sizeof(line3),
                        "%.1f",
                        sensor_data.humidity
                    );


                    ssd1306_set_cursor(
                        0,
                        2
                    );

                    ssd1306_write_text(
                        line2
                    );


                    ssd1306_set_cursor(
                        0,
                        4
                    );

                    ssd1306_write_text(
                        line3
                    );

                    break;


                case DisplayMode::LIGHT:

                    snprintf(
                        line2,
                        sizeof(line2),
                        "LIGHT"
                    );


                    snprintf(
                        line3,
                        sizeof(line3),
                        "%d%%",
                        sensor_data.lightLevel
                    );


                    ssd1306_set_cursor(
                        0,
                        2
                    );

                    ssd1306_write_text(
                        line2
                    );


                    ssd1306_set_cursor(
                        0,
                        4
                    );

                    ssd1306_write_text(
                        line3
                    );

                    break;


                case DisplayMode::MOTION:

                    snprintf(
                        line2,
                        sizeof(line2),
                        "MOTION"
                    );


                    snprintf(
                        line3,
                        sizeof(line3),
                        "%s",
                        sensor_data.motionDetected
                            ? "DETECTED"
                            : "NO MOTION"
                    );


                    ssd1306_set_cursor(
                        0,
                        2
                    );

                    ssd1306_write_text(
                        line2
                    );


                    ssd1306_set_cursor(
                        0,
                        4
                    );

                    ssd1306_write_text(
                        line3
                    );

                    break;
            }


            /* =============================================
             * SEND BUFFER TO OLED
             * ============================================= */

            esp_err_t result =
                ssd1306_update();


            if (result != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "OLED update failed: %s",
                    esp_err_to_name(result)
                );
            }
        }
    }
}


/* =========================================================
 * DISPLAY MODE NAME
 * ========================================================= */

static const char *displayModeName(
    DisplayMode mode
)
{
    switch (mode)
    {
        case DisplayMode::TEMPERATURE:
            return "TEMPERATURE";

        case DisplayMode::HUMIDITY:
            return "HUMIDITY";

        case DisplayMode::LIGHT:
            return "LIGHT";

        case DisplayMode::MOTION:
            return "MOTION";

        default:
            return "UNKNOWN";
    }
}