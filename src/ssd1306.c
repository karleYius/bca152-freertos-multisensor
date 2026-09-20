#include "ssd1306.h"

#include <stdint.h>
#include <string.h>

static i2c_master_dev_handle_t oled_device;

static uint8_t cursor_x = 0;
static uint8_t cursor_y = 0;

static uint8_t display_buffer[
    SSD1306_WIDTH * SSD1306_HEIGHT / 8
];

/*
 * Simple 5x7 font.
 * Each character is 5 columns wide.
 */
static const uint8_t font_5x7[][5] = {

    /* Space */
    [0] = {0x00, 0x00, 0x00, 0x00, 0x00},

    /* A */
    ['A' - 32] = {0x7E, 0x11, 0x11, 0x11, 0x7E},

    /* B */
    ['B' - 32] = {0x7F, 0x49, 0x49, 0x49, 0x36},

    /* C */
    ['C' - 32] = {0x3E, 0x41, 0x41, 0x41, 0x22},

    /* D */
    ['D' - 32] = {0x7F, 0x41, 0x41, 0x22, 0x1C},

    /* E */
    ['E' - 32] = {0x7F, 0x49, 0x49, 0x49, 0x41},

    /* F */
    ['F' - 32] = {0x7F, 0x09, 0x09, 0x09, 0x01},

    /* G */
    ['G' - 32] = {0x3E, 0x41, 0x49, 0x49, 0x7A},

    /* H */
    ['H' - 32] = {0x7F, 0x08, 0x08, 0x08, 0x7F},

    /* I */
    ['I' - 32] = {0x00, 0x41, 0x7F, 0x41, 0x00},

    /* J */
    ['J' - 32] = {0x20, 0x40, 0x41, 0x3F, 0x01},

    /* K */
    ['K' - 32] = {0x7F, 0x08, 0x14, 0x22, 0x41},

    /* L */
    ['L' - 32] = {0x7F, 0x40, 0x40, 0x40, 0x40},

    /* M */
    ['M' - 32] = {0x7F, 0x02, 0x0C, 0x02, 0x7F},

    /* N */
    ['N' - 32] = {0x7F, 0x04, 0x08, 0x10, 0x7F},

    /* O */
    ['O' - 32] = {0x3E, 0x41, 0x41, 0x41, 0x3E},

    /* P */
    ['P' - 32] = {0x7F, 0x09, 0x09, 0x09, 0x06},

    /* Q */
    ['Q' - 32] = {0x3E, 0x41, 0x51, 0x21, 0x5E},

    /* R */
    ['R' - 32] = {0x7F, 0x09, 0x19, 0x29, 0x46},

    /* S */
    ['S' - 32] = {0x46, 0x49, 0x49, 0x49, 0x31},

    /* T */
    ['T' - 32] = {0x01, 0x01, 0x7F, 0x01, 0x01},

    /* U */
    ['U' - 32] = {0x3F, 0x40, 0x40, 0x40, 0x3F},

    /* V */
    ['V' - 32] = {0x1F, 0x20, 0x40, 0x20, 0x1F},

    /* W */
    ['W' - 32] = {0x3F, 0x40, 0x38, 0x40, 0x3F},

    /* X */
    ['X' - 32] = {0x63, 0x14, 0x08, 0x14, 0x63},

    /* Y */
    ['Y' - 32] = {0x07, 0x08, 0x70, 0x08, 0x07},

    /* Z */
    ['Z' - 32] = {0x61, 0x51, 0x49, 0x45, 0x43},

    /* 0 */
    ['0' - 32] = {0x3E, 0x45, 0x49, 0x51, 0x3E},

    /* 1 */
    ['1' - 32] = {0x00, 0x41, 0x7F, 0x40, 0x00},

    /* 2 */
    ['2' - 32] = {0x42, 0x61, 0x51, 0x49, 0x46},

    /* 3 */
    ['3' - 32] = {0x21, 0x41, 0x45, 0x4B, 0x31},

    /* 4 */
    ['4' - 32] = {0x18, 0x14, 0x12, 0x7F, 0x10},

    /* 5 */
    ['5' - 32] = {0x27, 0x45, 0x45, 0x45, 0x39},

    /* 6 */
    ['6' - 32] = {0x3C, 0x4A, 0x49, 0x49, 0x30},

    /* 7 */
    ['7' - 32] = {0x01, 0x71, 0x09, 0x05, 0x03},

    /* 8 */
    ['8' - 32] = {0x36, 0x49, 0x49, 0x49, 0x36},

    /* 9 */
    ['9' - 32] = {0x06, 0x49, 0x49, 0x29, 0x1E},

    /* Period */
    ['.' - 32] = {0x00, 0x60, 0x60, 0x00, 0x00},

    /* Colon */
    [':' - 32] = {0x00, 0x36, 0x36, 0x00, 0x00}
};


/*
 * Send one command to the SSD1306.
 */
static esp_err_t ssd1306_command(uint8_t command)
{
    uint8_t data[2];

    data[0] = 0x00;
    data[1] = command;

    return i2c_master_transmit(
        oled_device,
        data,
        sizeof(data),
        1000
    );
}


/*
 * Initialize the SSD1306 OLED.
 */
esp_err_t ssd1306_init(
    i2c_master_bus_handle_t bus_handle,
    uint8_t i2c_address
)
{
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = 400000,
    };

    esp_err_t result = i2c_master_bus_add_device(
        bus_handle,
        &device_config,
        &oled_device
    );

    if (result != ESP_OK)
    {
        return result;
    }

    /* SSD1306 initialization commands */

    ssd1306_command(0xAE); // Display OFF

    ssd1306_command(0xD5); // Set display clock
    ssd1306_command(0x80);

    ssd1306_command(0xA8); // Multiplex ratio
    ssd1306_command(0x3F);

    ssd1306_command(0xD3); // Display offset
    ssd1306_command(0x00);

    ssd1306_command(0x40); // Start line

    ssd1306_command(0x8D); // Charge pump
    ssd1306_command(0x14);

    ssd1306_command(0x20); // Memory addressing mode
    ssd1306_command(0x00);

    ssd1306_command(0xA1); // Segment remap
    ssd1306_command(0xC8); // COM scan direction

    ssd1306_command(0xDA); // COM pins
    ssd1306_command(0x12);

    ssd1306_command(0x81); // Contrast
    ssd1306_command(0x7F);

    ssd1306_command(0xD9); // Pre-charge
    ssd1306_command(0xF1);

    ssd1306_command(0xDB); // VCOMH
    ssd1306_command(0x40);

    ssd1306_command(0xA4); // Display follows RAM
    ssd1306_command(0xA6); // Normal display

    ssd1306_command(0xAF); // Display ON

    /* Clear display buffer */

    memset(
        display_buffer,
        0,
        sizeof(display_buffer)
    );

    cursor_x = 0;
    cursor_y = 0;

    return ssd1306_update();
}


/*
 * Clear the display buffer.
 */
esp_err_t ssd1306_clear(void)
{
    memset(
        display_buffer,
        0,
        sizeof(display_buffer)
    );

    cursor_x = 0;
    cursor_y = 0;

    return ESP_OK;
}


/*
 * Set the software cursor position.
 *
 * x = horizontal position
 * y = page number (0-7)
 */
esp_err_t ssd1306_set_cursor(
    uint8_t x,
    uint8_t y
)
{
    if (x >= SSD1306_WIDTH || y >= 8)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cursor_x = x;
    cursor_y = y;

    return ESP_OK;
}


/*
 * Write text into the display buffer.
 */
esp_err_t ssd1306_write_text(
    const char *text
)
{
    if (text == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    while (*text)
    {
        char character = *text;

        /*
         * Move to the beginning of the next line.
         */
        if (character == '\n')
        {
            cursor_x = 0;
            cursor_y++;

            text++;
            continue;
        }

        /*
         * Ignore unsupported characters.
         */
        if (character < 32 || character > 90)
        {
            text++;
            continue;
        }

        /*
         * Get the 5x7 font data.
         */
        const uint8_t *glyph =
            font_5x7[character - 32];

        /*
         * Move to the next line if there
         * is not enough horizontal space.
         */
        if (cursor_x + 6 >= SSD1306_WIDTH)
        {
            cursor_x = 0;
            cursor_y++;
        }

        /*
         * Stop if we reached the bottom.
         */
        if (cursor_y >= 8)
        {
            break;
        }

        /*
         * Copy the character into the display buffer.
         */
        for (int column = 0; column < 5; column++)
        {
            display_buffer[
                cursor_y * SSD1306_WIDTH
                + cursor_x
                + column
            ] = glyph[column];
        }

        /*
         * Move cursor 6 pixels:
         * 5 pixels for the character
         * 1 pixel for spacing
         */
        cursor_x += 6;

        text++;
    }

    return ESP_OK;
}


/*
 * Send the display buffer to the OLED.
 */
esp_err_t ssd1306_update(void)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        esp_err_t result;

        /*
         * Select page.
         */
        result = ssd1306_command(
            0xB0 + page
        );

        if (result != ESP_OK)
        {
            return result;
        }

        /*
         * Set lower column address.
         */
        result = ssd1306_command(0x00);

        if (result != ESP_OK)
        {
            return result;
        }

        /*
         * Set higher column address.
         */
        result = ssd1306_command(0x10);

        if (result != ESP_OK)
        {
            return result;
        }

        /*
         * First byte is 0x40,
         * meaning the following bytes are display data.
         */
        uint8_t data[
            1 + SSD1306_WIDTH
        ];

        data[0] = 0x40;

        memcpy(
            &data[1],
            &display_buffer[
                page * SSD1306_WIDTH
            ],
            SSD1306_WIDTH
        );

        /*
         * Send this page to the OLED.
         */
        result = i2c_master_transmit(
            oled_device,
            data,
            sizeof(data),
            1000
        );

        if (result != ESP_OK)
        {
            return result;
        }
    }

    return ESP_OK;
}