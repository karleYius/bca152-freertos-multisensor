#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"

#include "esp_adc/adc_oneshot.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "dht22.h"
#include "ssd1306.h"
#include "alarm_logic.h"


/* =========================================================
 * GPIO / HARDWARE CONFIGURATION
 * ========================================================= */

#define DHT22_GPIO              GPIO_NUM_4

#define LDR_CHANNEL             ADC_CHANNEL_6
#define LDR_GPIO                GPIO_NUM_34

#define I2C_SDA_GPIO            GPIO_NUM_21
#define I2C_SCL_GPIO            GPIO_NUM_22
#define OLED_I2C_ADDRESS        0x3C

/* Rotary Encoder */
#define ENCODER_CLK_GPIO        GPIO_NUM_18
#define ENCODER_DT_GPIO         GPIO_NUM_19
#define ENCODER_SW_GPIO         GPIO_NUM_23

/* PIR Motion Sensor */
#define PIR_GPIO                GPIO_NUM_27


/* =========================================================
 * TIMING
 * ========================================================= */

#define SENSOR_INTERVAL_MS      2000
#define MOTION_CHECK_INTERVAL_MS 100
#define ENCODER_CHECK_INTERVAL_MS 5

/* Laboratory testing timeout */
#define INACTIVITY_TIMEOUT_MS   15000


/* =========================================================
 * PART X — EVENT GROUP BITS
 * ========================================================= */

#define EVENT_ACTIVE    BIT0
#define EVENT_MOTION    BIT1
#define EVENT_ALARM     BIT2


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
 * SYSTEM STATE
 * ========================================================= */

enum class SystemState
{
    ACTIVE,
    INACTIVE
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

/* ADC */
static adc_oneshot_unit_handle_t adc_handle = NULL;

/* I2C */
static i2c_master_bus_handle_t i2c_bus_handle = NULL;

/* Sensor queue */
static QueueHandle_t sensor_queue = NULL;

/* PART X Event Group */
static EventGroupHandle_t system_event_group = NULL;

/* Mutex for shared state */
static SemaphoreHandle_t state_mutex = NULL;

/* Current OLED page */
static DisplayMode currentMode =
    DisplayMode::TEMPERATURE;

/* ACTIVE / INACTIVE state */
static SystemState systemState =
    SystemState::ACTIVE;

/* Latest motion state */
static bool motionDetected = false;


/* =========================================================
 * FUNCTION DECLARATIONS
 * ========================================================= */

static void sensor_task(void *pvParameters);

static void input_task(void *pvParameters);

static void display_task(void *pvParameters);

static void motion_task(void *pvParameters);

static const char *displayModeName(
    DisplayMode mode
);

static bool isSystemActive(void);


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

    channel_config.bitwidth =
        ADC_BITWIDTH_DEFAULT;

    channel_config.atten =
        ADC_ATTEN_DB_12;


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

    i2c_config.i2c_port =
        I2C_NUM_0;

    i2c_config.sda_io_num =
        I2C_SDA_GPIO;

    i2c_config.scl_io_num =
        I2C_SCL_GPIO;

    i2c_config.clk_source =
        I2C_CLK_SRC_DEFAULT;

    i2c_config.glitch_ignore_cnt =
        7;

    i2c_config.flags.enable_internal_pullup =
        true;


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

    encoder_config.mode =
        GPIO_MODE_INPUT;

    encoder_config.pull_up_en =
        GPIO_PULLUP_ENABLE;

    encoder_config.pull_down_en =
        GPIO_PULLDOWN_DISABLE;

    encoder_config.intr_type =
        GPIO_INTR_DISABLE;


    ESP_ERROR_CHECK(
        gpio_config(
            &encoder_config
        )
    );


    ESP_LOGI(
        TAG,
        "Rotary encoder configured"
    );


    /* =====================================================
     * CONFIGURE PIR MOTION SENSOR
     * ===================================================== */

    gpio_config_t pir_config = {};

    pir_config.pin_bit_mask =
        (1ULL << PIR_GPIO);

    pir_config.mode =
        GPIO_MODE_INPUT;

    pir_config.pull_up_en =
        GPIO_PULLUP_DISABLE;

    pir_config.pull_down_en =
        GPIO_PULLDOWN_DISABLE;

    pir_config.intr_type =
        GPIO_INTR_DISABLE;


    ESP_ERROR_CHECK(
        gpio_config(
            &pir_config
        )
    );


    ESP_LOGI(
        TAG,
        "PIR motion sensor configured"
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
     * CREATE EVENT GROUP
     *
     * PART X
     *
     * EVENT_ACTIVE
     * EVENT_MOTION
     * EVENT_ALARM
     * ===================================================== */

    system_event_group =
        xEventGroupCreate();


    if (system_event_group == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to create event group"
        );

        return;
    }


    /*
     * System starts in ACTIVE state.
     *
     * EVENT_ACTIVE is therefore SET.
     */
    xEventGroupSetBits(
        system_event_group,
        EVENT_ACTIVE
    );


    ESP_LOGI(
        TAG,
        "Event group created"
    );


    /* =====================================================
     * CREATE STATE MUTEX
     * ===================================================== */

    state_mutex =
        xSemaphoreCreateMutex();


    if (state_mutex == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to create state mutex"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "State mutex created"
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


    /* =====================================================
     * START MOTION TASK
     * ===================================================== */

    BaseType_t motion_result =
        xTaskCreate(
            motion_task,
            "MotionTask",
            4096,
            NULL,
            6,
            NULL
        );


    if (motion_result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create MotionTask"
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
 *
 * Responsibilities:
 * - Read DHT22
 * - Read LDR
 * - Evaluate temperature alarm
 * - Send SensorData to queue
 * - Signal EVENT_ALARM
 * - Run periodically using vTaskDelayUntil()
 * ========================================================= */

static void sensor_task(
    void *pvParameters
)
{
    (void)pvParameters;


    TickType_t lastWakeTime =
        xTaskGetTickCount();


    float temperature = 0.0f;
    float humidity = 0.0f;

    int light_raw = 0;


    SensorData sensor_data =
    {
        0.0f,
        0.0f,
        0,
        false
    };


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
                esp_err_to_name(
                    dht_result
                )
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
            /*
             * Convert raw ADC value to
             * approximate percentage.
             */
            sensor_data.lightLevel =
                (light_raw * 100) / 4095;
        }
        else
        {
            printf(
                "LDR read failed: %s\n",
                esp_err_to_name(
                    ldr_result
                )
            );
        }


        /* =================================================
         * GET CURRENT MOTION STATE
         * ================================================= */

        if (
            xSemaphoreTake(
                state_mutex,
                pdMS_TO_TICKS(10)
            ) == pdTRUE
        )
        {
            sensor_data.motionDetected =
                motionDetected;

            xSemaphoreGive(
                state_mutex
            );
        }


        /* =================================================
         * EVALUATE TEMPERATURE ALARM
         * ================================================= */

        AlarmState alarmState =
            evaluateTemperature(
                sensor_data.temperature
            );


        /* =================================================
         * PART X — SIGNAL ALARM EVENT
         * ================================================= */

        if (
            alarmState ==
                AlarmState::LOW_TEMPERATURE ||
            alarmState ==
                AlarmState::HIGH_TEMPERATURE
        )
        {
            xEventGroupSetBits(
                system_event_group,
                EVENT_ALARM
            );
        }
        else
        {
            xEventGroupClearBits(
                system_event_group,
                EVENT_ALARM
            );
        }


        /* =================================================
         * PRINT ALARM STATE
         * ================================================= */

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
         * SEND DATA TO SENSOR QUEUE
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
         * SERIAL OUTPUT
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


        /* =================================================
         * SHOW CURRENT EVENT BITS
         * ================================================= */

        EventBits_t eventBits =
            xEventGroupGetBits(
                system_event_group
            );


        printf(
            "Events: ACTIVE=%s MOTION=%s ALARM=%s\n",
            (eventBits & EVENT_ACTIVE)
                ? "SET"
                : "CLEAR",

            (eventBits & EVENT_MOTION)
                ? "SET"
                : "CLEAR",

            (eventBits & EVENT_ALARM)
                ? "SET"
                : "CLEAR"
        );


        printf(
            "----------------------\n"
        );


        /* =================================================
         * PERIODIC EXECUTION
         * ================================================= */

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(
                SENSOR_INTERVAL_MS
            )
        );
    }
}


/* =========================================================
 * MOTION TASK
 *
 * Responsibilities:
 * - Monitor PIR
 * - Produce EVENT_MOTION
 * - Maintain ACTIVE / INACTIVE state
 * - Produce/clear EVENT_ACTIVE
 * - Restore ACTIVE when motion is detected
 * ========================================================= */

static void motion_task(
    void *pvParameters
)
{
    (void)pvParameters;


    TickType_t lastMotionTime =
        xTaskGetTickCount();


    while (1)
    {
        int pirLevel =
            gpio_get_level(
                PIR_GPIO
            );


        TickType_t now =
            xTaskGetTickCount();


        /* =================================================
         * MOTION DETECTED
         * ================================================= */

        if (pirLevel == 1)
        {
            lastMotionTime = now;


            /* ---------------------------------------------
             * Update shared motion state
             * --------------------------------------------- */

            if (
                xSemaphoreTake(
                    state_mutex,
                    pdMS_TO_TICKS(10)
                ) == pdTRUE
            )
            {
                motionDetected = true;

                xSemaphoreGive(
                    state_mutex
                );
            }


            /* ---------------------------------------------
             * PART X — SIGNAL MOTION EVENT
             * --------------------------------------------- */

            xEventGroupSetBits(
                system_event_group,
                EVENT_MOTION
            );


            /* ---------------------------------------------
             * INACTIVE -> ACTIVE
             * --------------------------------------------- */

            if (
                !isSystemActive()
            )
            {
                if (
                    xSemaphoreTake(
                        state_mutex,
                        pdMS_TO_TICKS(10)
                    ) == pdTRUE
                )
                {
                    systemState =
                        SystemState::ACTIVE;

                    xSemaphoreGive(
                        state_mutex
                    );
                }


                /*
                 * EVENT_ACTIVE represents the
                 * current system state.
                 */
                xEventGroupSetBits(
                    system_event_group,
                    EVENT_ACTIVE
                );


                printf(
                    "STATE: INACTIVE -> ACTIVE "
                    "(motion detected)\n"
                );


                printf(
                    "DISPLAY: ON "
                    "(system active)\n"
                );
            }
        }
        else
        {
            /* =================================================
             * NO CURRENT PIR SIGNAL
             * ================================================= */

            if (
                xSemaphoreTake(
                    state_mutex,
                    pdMS_TO_TICKS(10)
                ) == pdTRUE
            )
            {
                motionDetected = false;

                xSemaphoreGive(
                    state_mutex
                );
            }
        }


        /* =================================================
         * ACTIVE -> INACTIVE
         *
         * If no motion occurs for 15 seconds.
         * ================================================= */

        if (
            isSystemActive()
        )
        {
            TickType_t elapsed =
                now - lastMotionTime;


            if (
                elapsed >=
                pdMS_TO_TICKS(
                    INACTIVITY_TIMEOUT_MS
                )
            )
            {
                if (
                    xSemaphoreTake(
                        state_mutex,
                        pdMS_TO_TICKS(10)
                    ) == pdTRUE
                )
                {
                    systemState =
                        SystemState::INACTIVE;

                    xSemaphoreGive(
                        state_mutex
                    );
                }


                /*
                 * Clear ACTIVE event bit.
                 */
                xEventGroupClearBits(
                    system_event_group,
                    EVENT_ACTIVE
                );


                printf(
                    "STATE: ACTIVE -> INACTIVE "
                    "(15s inactivity)\n"
                );


                printf(
                    "DISPLAY: OFF "
                    "(system inactive)\n"
                );
            }
        }


        /* =================================================
         * MOTION TASK DELAY
         * ================================================= */

        vTaskDelay(
            pdMS_TO_TICKS(
                MOTION_CHECK_INTERVAL_MS
            )
        );
    }
}


/* =========================================================
 * INPUT TASK
 *
 * Rotary encoder:
 *
 * Clockwise:
 *
 * TEMPERATURE
 *      ↓
 * HUMIDITY
 *      ↓
 * LIGHT
 *      ↓
 * MOTION
 *      ↓
 * TEMPERATURE
 *
 *
 * Counterclockwise:
 *
 * TEMPERATURE
 *      ↑
 * HUMIDITY
 *      ↑
 * LIGHT
 *      ↑
 * MOTION
 *      ↑
 * TEMPERATURE
 *
 * Encoder is active only while the system is ACTIVE.
 * ========================================================= */

static void input_task(
    void *pvParameters
)
{
    (void)pvParameters;


    int previousCLK =
        gpio_get_level(
            ENCODER_CLK_GPIO
        );


    TickType_t lastRotationTime = 0;


    while (1)
    {
        /* =================================================
         * ENCODER DISABLED WHILE INACTIVE
         * ================================================= */

        if (
            !isSystemActive()
        )
        {
            vTaskDelay(
                pdMS_TO_TICKS(
                    ENCODER_CHECK_INTERVAL_MS
                )
            );

            continue;
        }


        int currentCLK =
            gpio_get_level(
                ENCODER_CLK_GPIO
            );


        /* =================================================
         * DETECT ROTATION
         *
         * Falling edge of CLK
         * ================================================= */

        if (
            previousCLK == 1 &&
            currentCLK == 0
        )
        {
            TickType_t now =
                xTaskGetTickCount();


            /* =================================================
             * DEBOUNCE
             * ================================================= */

            if (
                (now - lastRotationTime) >=
                pdMS_TO_TICKS(50)
            )
            {
                int dt =
                    gpio_get_level(
                        ENCODER_DT_GPIO
                    );


                /* =================================================
                 * CLOCKWISE
                 * ================================================= */

                if (dt == 1)
                {
                    if (
                        xSemaphoreTake(
                            state_mutex,
                            pdMS_TO_TICKS(10)
                        ) == pdTRUE
                    )
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


                        xSemaphoreGive(
                            state_mutex
                        );
                    }
                }


                /* =================================================
                 * COUNTERCLOCKWISE
                 * ================================================= */

                else
                {
                    if (
                        xSemaphoreTake(
                            state_mutex,
                            pdMS_TO_TICKS(10)
                        ) == pdTRUE
                    )
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


                        xSemaphoreGive(
                            state_mutex
                        );
                    }
                }


                lastRotationTime = now;
            }
        }


        previousCLK = currentCLK;


        /* =================================================
         * SMALL POLLING DELAY
         * ================================================= */

        vTaskDelay(
            pdMS_TO_TICKS(
                ENCODER_CHECK_INTERVAL_MS
            )
        );
    }
}


/* =========================================================
 * DISPLAY TASK
 *
 * Responsibilities:
 * - Own the OLED
 * - Consume SensorData queue
 * - Consume/observe Event Group
 * - Turn display off while INACTIVE
 * ========================================================= */

static void display_task(
    void *pvParameters
)
{
    (void)pvParameters;


    SensorData sensor_data =
    {
        0.0f,
        0.0f,
        0,
        false
    };


    bool displayWasActive = true;

    bool previousAlarmState = false;


    while (1)
    {
        /* =================================================
         * CHECK SYSTEM STATE
         * ================================================= */

        bool active =
            isSystemActive();


        /* =================================================
         * INACTIVE BEHAVIOR
         *
         * OLED blank
         * Display operations reduced
         * MotionTask continues running
         * ================================================= */

        if (!active)
        {
            if (displayWasActive)
            {
                /*
                 * Clear the OLED once when entering
                 * INACTIVE.
                 */
                ssd1306_clear();

                ssd1306_update();

                displayWasActive = false;


                printf(
                    "DISPLAY: OFF "
                    "(system inactive)\n"
                );
            }


            vTaskDelay(
                pdMS_TO_TICKS(500)
            );

            continue;
        }


        /* =================================================
         * SYSTEM IS ACTIVE
         * ================================================= */

        if (!displayWasActive)
        {
            displayWasActive = true;

            printf(
                "DISPLAY: ON "
                "(system active)\n"
            );
        }


        /* =================================================
         * WAIT FOR SENSOR DATA
         * ================================================= */

        if (
            xQueueReceive(
                sensor_queue,
                &sensor_data,
                pdMS_TO_TICKS(2500)
            ) == pdPASS
        )
        {
            /* =================================================
             * READ EVENT GROUP
             * ================================================= */

            EventBits_t eventBits =
                xEventGroupGetBits(
                    system_event_group
                );


            /* =================================================
             * EVENT_ACTIVE
             *
             * This is a state bit.
             * Do NOT clear it here.
             *
             * MotionTask owns this bit.
             * ================================================= */

            bool activeEvent =
                (eventBits & EVENT_ACTIVE) != 0;


            /* =================================================
             * EVENT_MOTION
             *
             * This is a transient event.
             * DisplayTask consumes and clears it.
             * ================================================= */

            if (
                eventBits & EVENT_MOTION
            )
            {
                printf(
                    "EVENT: MOTION DETECTED\n"
                );


                xEventGroupClearBits(
                    system_event_group,
                    EVENT_MOTION
                );
            }


            /* =================================================
             * EVENT_ALARM
             *
             * This is a state/event bit.
             * SensorTask owns it.
             * DisplayTask only observes it.
             * ================================================= */

            bool alarmEvent =
                (eventBits & EVENT_ALARM) != 0;


            if (
                alarmEvent !=
                previousAlarmState
            )
            {
                if (alarmEvent)
                {
                    printf(
                        "EVENT: ALARM ACTIVE\n"
                    );
                }
                else
                {
                    printf(
                        "EVENT: ALARM CLEARED\n"
                    );
                }


                previousAlarmState =
                    alarmEvent;
            }


            /* =================================================
             * DISPLAY
             * ================================================= */

            ssd1306_clear();


            /* =================================================
             * HEADER
             * ================================================= */

            ssd1306_set_cursor(
                0,
                0
            );

            ssd1306_write_text(
                "ROOM MONITOR"
            );


            /* =================================================
             * DISPLAY SELECTED PAGE
             * ================================================= */

            char line2[32];
            char line3[32];


            switch (currentMode)
            {
                /* =================================================
                 * TEMPERATURE
                 * ================================================= */

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


                /* =================================================
                 * HUMIDITY
                 * ================================================= */

                case DisplayMode::HUMIDITY:

                    snprintf(
                        line2,
                        sizeof(line2),
                        "HUMIDITY"
                    );


                    snprintf(
                        line3,
                        sizeof(line3),
                        "%.1f %%",
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


                /* =================================================
                 * LIGHT
                 * ================================================= */

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


                /* =================================================
                 * MOTION
                 * ================================================= */

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


            /* =================================================
             * UPDATE OLED
             * ================================================= */

            esp_err_t result =
                ssd1306_update();


            if (result != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "OLED update failed: %s",
                    esp_err_to_name(
                        result
                    )
                );
            }


            /*
             * activeEvent is intentionally read so that
             * EVENT_ACTIVE is a meaningful consumed/observed
             * system-state signal.
             */
            (void)activeEvent;
        }
    }
}


/* =========================================================
 * CHECK SYSTEM STATE
 *
 * EVENT_ACTIVE is the authoritative event-group state bit.
 * ========================================================= */

static bool isSystemActive(void)
{
    if (
        system_event_group == NULL
    )
    {
        return true;
    }


    EventBits_t bits =
        xEventGroupGetBits(
            system_event_group
        );


    return (
        (bits & EVENT_ACTIVE) != 0
    );
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