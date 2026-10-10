/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "iot_button.h"
#include "esp_err.h"
#include "esp_private/usb_phy.h"

#define BUTTON_1 (GPIO_NUM_12)
#define BUTTON_2 (GPIO_NUM_13)
static const char *TAG = "HID";

/* ------------------------------------------------------------------ */
/* HID descriptors                                                     */
/* ------------------------------------------------------------------ */
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))};

static const char *hid_string_descriptor[5] = {
    (char[]){0x09, 0x04}, // 0: supported language = English (0x0409)
    "Akshay",             // 1: manufacturer
    "ESP32-S3 Keyboard",  // 2: product
    "123456",             // 3: serial
    "HID keyboard",       // 4: HID interface
};

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/* ------------------------------------------------------------------ */
/* TinyUSB HID callbacks (required)                                    */
/* ------------------------------------------------------------------ */
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}

/* ------------------------------------------------------------------ */
/* TinyUSB start / stop                                                */
/* ------------------------------------------------------------------ */
static usb_phy_handle_t s_jtag_phy;

static esp_err_t usb_tinyusb_start(void)
{
    ESP_LOGI(TAG, "USB initialization");

    // If we handed the PHY to Serial/JTAG earlier, release it first
    if (s_jtag_phy)
    {
        usb_del_phy(s_jtag_phy);
        s_jtag_phy = NULL;
    }

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();

    tusb_cfg.descriptor.device = NULL;
    tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
    tusb_cfg.descriptor.string = hid_string_descriptor;
    tusb_cfg.descriptor.string_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]);
#if (TUD_OPT_HIGH_SPEED)
    tusb_cfg.descriptor.high_speed_config = hid_configuration_descriptor;
#endif // TUD_OPT_HIGH_SPEED

    esp_err_t err = tinyusb_driver_install(&tusb_cfg);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "USB initialization DONE");
    }
    else
    {
        ESP_LOGE(TAG, "tinyusb_driver_install failed: %s", esp_err_to_name(err));
    }
    return err;
}

static void usb_tinyusb_stop(void)
{
    ESP_ERROR_CHECK(tinyusb_driver_uninstall());

    // Hand the PHY back to USB-Serial-JTAG
    usb_phy_config_t cfg = {
        .controller = USB_PHY_CTRL_SERIAL_JTAG,
        .target = USB_PHY_TARGET_INT,
    };
    ESP_ERROR_CHECK(usb_new_phy(&cfg, &s_jtag_phy));
    ESP_LOGI(TAG, "Back on JTAG");
}

/* ------------------------------------------------------------------ */
/* Send keys                                                           */
/* ------------------------------------------------------------------ */

// Lookup table structure linking ASCII to HID Keycode + Modifier
typedef struct
{
    uint8_t modifier;
    uint8_t keycode;
} ascii_to_hid_t;

// Shift modifier shorthand
#define SHIFT KEYBOARD_MODIFIER_LEFTSHIFT

// ASCII lookup table (Indices 32 to 126 map to standard printable characters)
static const ascii_to_hid_t ascii_hid_table[128] = {
    [' '] = {0, HID_KEY_SPACE},
    ['!'] = {SHIFT, HID_KEY_1},
    ['"'] = {SHIFT, HID_KEY_APOSTROPHE},
    ['#'] = {SHIFT, HID_KEY_3},
    ['$'] = {SHIFT, HID_KEY_4},
    ['%'] = {SHIFT, HID_KEY_5},
    ['&'] = {SHIFT, HID_KEY_7},
    ['\''] = {0, HID_KEY_APOSTROPHE},
    ['('] = {SHIFT, HID_KEY_9},
    [')'] = {SHIFT, HID_KEY_0},
    ['*'] = {SHIFT, HID_KEY_8},
    ['+'] = {SHIFT, HID_KEY_EQUAL},
    [','] = {0, HID_KEY_COMMA},
    ['-'] = {0, HID_KEY_MINUS},
    ['.'] = {0, HID_KEY_PERIOD},
    ['/'] = {0, HID_KEY_SLASH},

    // Numbers 0-9
    ['0'] = {0, HID_KEY_0},
    ['1'] = {0, HID_KEY_1},
    ['2'] = {0, HID_KEY_2},
    ['3'] = {0, HID_KEY_3},
    ['4'] = {0, HID_KEY_4},
    ['5'] = {0, HID_KEY_5},
    ['6'] = {0, HID_KEY_6},
    ['7'] = {0, HID_KEY_7},
    ['8'] = {0, HID_KEY_8},
    ['9'] = {0, HID_KEY_9},

    [':'] = {SHIFT, HID_KEY_SEMICOLON},
    [';'] = {0, HID_KEY_SEMICOLON},
    ['<'] = {SHIFT, HID_KEY_COMMA},
    ['='] = {0, HID_KEY_EQUAL},
    ['>'] = {SHIFT, HID_KEY_PERIOD},
    ['?'] = {SHIFT, HID_KEY_SLASH},
    ['@'] = {SHIFT, HID_KEY_2},

    // Uppercase A-Z
    ['A'] = {SHIFT, HID_KEY_A},
    ['B'] = {SHIFT, HID_KEY_B},
    ['C'] = {SHIFT, HID_KEY_C},
    ['D'] = {SHIFT, HID_KEY_D},
    ['E'] = {SHIFT, HID_KEY_E},
    ['F'] = {SHIFT, HID_KEY_F},
    ['G'] = {SHIFT, HID_KEY_G},
    ['H'] = {SHIFT, HID_KEY_H},
    ['I'] = {SHIFT, HID_KEY_I},
    ['J'] = {SHIFT, HID_KEY_J},
    ['K'] = {SHIFT, HID_KEY_K},
    ['L'] = {SHIFT, HID_KEY_L},
    ['M'] = {SHIFT, HID_KEY_M},
    ['N'] = {SHIFT, HID_KEY_N},
    ['O'] = {SHIFT, HID_KEY_O},
    ['P'] = {SHIFT, HID_KEY_P},
    ['Q'] = {SHIFT, HID_KEY_Q},
    ['R'] = {SHIFT, HID_KEY_R},
    ['S'] = {SHIFT, HID_KEY_S},
    ['T'] = {SHIFT, HID_KEY_T},
    ['U'] = {SHIFT, HID_KEY_U},
    ['V'] = {SHIFT, HID_KEY_V},
    ['W'] = {SHIFT, HID_KEY_W},
    ['X'] = {SHIFT, HID_KEY_X},
    ['Y'] = {SHIFT, HID_KEY_Y},
    ['Z'] = {SHIFT, HID_KEY_Z},

    ['['] = {0, HID_KEY_BRACKET_LEFT},
    ['\\'] = {0, HID_KEY_BACKSLASH},
    [']'] = {0, HID_KEY_BRACKET_RIGHT},
    ['^'] = {SHIFT, HID_KEY_6},
    ['_'] = {SHIFT, HID_KEY_MINUS},
    ['`'] = {0, HID_KEY_GRAVE},

    // Lowercase a-z
    ['a'] = {0, HID_KEY_A},
    ['b'] = {0, HID_KEY_B},
    ['c'] = {0, HID_KEY_C},
    ['d'] = {0, HID_KEY_D},
    ['e'] = {0, HID_KEY_E},
    ['f'] = {0, HID_KEY_F},
    ['g'] = {0, HID_KEY_G},
    ['h'] = {0, HID_KEY_H},
    ['i'] = {0, HID_KEY_I},
    ['j'] = {0, HID_KEY_J},
    ['k'] = {0, HID_KEY_K},
    ['l'] = {0, HID_KEY_L},
    ['m'] = {0, HID_KEY_M},
    ['n'] = {0, HID_KEY_N},
    ['o'] = {0, HID_KEY_O},
    ['p'] = {0, HID_KEY_P},
    ['q'] = {0, HID_KEY_Q},
    ['r'] = {0, HID_KEY_R},
    ['s'] = {0, HID_KEY_S},
    ['t'] = {0, HID_KEY_T},
    ['u'] = {0, HID_KEY_U},
    ['v'] = {0, HID_KEY_V},
    ['w'] = {0, HID_KEY_W},
    ['x'] = {0, HID_KEY_X},
    ['y'] = {0, HID_KEY_Y},
    ['z'] = {0, HID_KEY_Z},

    ['{'] = {SHIFT, HID_KEY_BRACKET_LEFT},
    ['|'] = {SHIFT, HID_KEY_BACKSLASH},
    ['}'] = {SHIFT, HID_KEY_BRACKET_RIGHT},
    ['~'] = {SHIFT, HID_KEY_GRAVE},

    // Support Control Characters
    ['\n'] = {0, HID_KEY_ENTER},
    ['\t'] = {0, HID_KEY_TAB},
    ['\b'] = {0, HID_KEY_BACKSPACE}};

static bool wait_hid_ready(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    while (!tud_hid_ready())
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms))
        {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return true;
}

// Reusable low-level keystroke delivery block
static void app_send_hid_character(uint8_t modifier, uint8_t keycode)
{
    uint8_t keys[6] = {keycode};

    if (!wait_hid_ready(200))
        return;
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, modifier, keys); // press
    vTaskDelay(pdMS_TO_TICKS(15));

    if (!wait_hid_ready(200))
        return;
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, modifier, NULL); // release
    vTaskDelay(pdMS_TO_TICKS(15));
}

// Main API: Iterates through any dynamic C-String and pushes reports
static void app_dynamic_hid_print(const char *str)
{
    while (*str)
    {
        uint8_t ch = (uint8_t)*str;

        // Ensure character falls inside safe ASCII structural bounds
        if (ch < 128 && (ascii_hid_table[ch].keycode != 0 || ch == ' '))
        {
            app_send_hid_character(ascii_hid_table[ch].modifier, ascii_hid_table[ch].keycode);
        }
        str++;
    }
}

static void app_login_creds()
{
}

static void app_protection_release_creds()
{
    // Press ALT to enter menu bar
    uint8_t keycode_arr[6] = {HID_KEY_ALT_LEFT};
    if (!wait_hid_ready(200))
        return;
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode_arr);
    vTaskDelay(pdMS_TO_TICKS(15));

    // Release Key
    if (!wait_hid_ready(200))
        return;
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
    vTaskDelay(pdMS_TO_TICKS(15));

    // Go through the menu bar
    app_dynamic_hid_print("CET");

    // Enter Password and press ENTER
    app_dynamic_hid_print("SecretPass\t\n");
}

/* ------------------------------------------------------------------ */
/* Button handling                                                     */
/* ------------------------------------------------------------------ */
static void on_button_press(int button)
{
    if (usb_tinyusb_start() != ESP_OK)
    {
        return;
    }

    // wait up to 5 s for the host to enumerate
    for (int i = 0; i < 50 && !tud_mounted(); i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (tud_mounted())
    {
        ESP_LOGI(TAG, "Mounted, sending keys");
        switch (button)
        {
        case 1:
            app_login_creds();
            break;
        case 2:
            app_protection_release_creds();
            break;
        default:
            ESP_LOGI(TAG, "Not valid button");
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // let the last report flush
    }
    else
    {
        ESP_LOGW(TAG, "Host did not mount in time");
    }

    usb_tinyusb_stop();
}

static void handle_button1()
{
    if (gpio_get_level(BUTTON_1) == 0)
    {                                  // pressed (active low)
        vTaskDelay(pdMS_TO_TICKS(30)); // debounce
        if (gpio_get_level(BUTTON_1) != 0)
        {
            return;
        }
        ESP_LOGI(TAG, "Button 1 Pressed");
        on_button_press(1);
        // wait for release so we don't retrigger
        while (gpio_get_level(BUTTON_1) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void handle_button2()
{
    if (gpio_get_level(BUTTON_2) == 0)
    {                                  // pressed (active low)
        vTaskDelay(pdMS_TO_TICKS(30)); // debounce
        if (gpio_get_level(BUTTON_2) != 0)
        {
            return;
        }
        ESP_LOGI(TAG, "Button 2 Pressed");
        on_button_press(2);
        // wait for release so we don't retrigger
        while (gpio_get_level(BUTTON_2) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void button_setup()
{
    // GPIO 13, 12 connected to 2 buttons
    const gpio_config_t button1_config = {
        .pin_bit_mask = BIT64(BUTTON_1),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button1_config));

    const gpio_config_t button2_config = {
        .pin_bit_mask = BIT64(BUTTON_2),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button2_config));
}

static void button_task(void *arg)
{
    (void)arg;

    button_setup();

    while (1)
    {
        handle_button1();
        handle_button2();
        // ESP_LOGI(TAG, "loop");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    xTaskCreate(button_task, "btn", 4096, NULL, 5, NULL);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}