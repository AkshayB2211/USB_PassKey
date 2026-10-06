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

#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *TAG = "example";

/* Flag to indicate if the host has suspended the USB bus */
static bool suspended = false;
/* Flag of possibility to Wakeup Host via Remote Wakeup feature */
static bool wakeup_host = false;

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
};

/**
 * @brief String descriptor
 */
const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
}

/********* Application ***************/

typedef enum {
    MOUSE_DIR_RIGHT,
    MOUSE_DIR_DOWN,
    MOUSE_DIR_LEFT,
    MOUSE_DIR_UP,
    MOUSE_DIR_MAX,
} mouse_dir_t;

#define DISTANCE_MAX        125
#define DELTA_SCALAR        5

static void mouse_draw_square_next_delta(int8_t *delta_x_ret, int8_t *delta_y_ret)
{
    static mouse_dir_t cur_dir = MOUSE_DIR_RIGHT;
    static uint32_t distance = 0;

    // Calculate next delta
    if (cur_dir == MOUSE_DIR_RIGHT) {
        *delta_x_ret = DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_DOWN) {
        *delta_x_ret = 0;
        *delta_y_ret = DELTA_SCALAR;
    } else if (cur_dir == MOUSE_DIR_LEFT) {
        *delta_x_ret = -DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_UP) {
        *delta_x_ret = 0;
        *delta_y_ret = -DELTA_SCALAR;
    }

    // Update cumulative distance for current direction
    distance += DELTA_SCALAR;
    // Check if we need to change direction
    if (distance >= DISTANCE_MAX) {
        distance = 0;
        cur_dir++;
        if (cur_dir == MOUSE_DIR_MAX) {
            cur_dir = 0;
        }
    }
}

// Lookup table structure linking ASCII to HID Keycode + Modifier
typedef struct {
    uint8_t modifier;
    uint8_t keycode;
} ascii_to_hid_t;

// Shift modifier shorthand
#define SHIFT KEYBOARD_MODIFIER_LEFTSHIFT

// ASCII lookup table (Indices 32 to 126 map to standard printable characters)
static const ascii_to_hid_t ascii_hid_table[128] = {
    [' ']  = { 0,     HID_KEY_SPACE },
    ['!']  = { SHIFT, HID_KEY_1 },
    ['"']  = { SHIFT, HID_KEY_APOSTROPHE },
    ['#']  = { SHIFT, HID_KEY_3 },
    ['$']  = { SHIFT, HID_KEY_4 },
    ['%']  = { SHIFT, HID_KEY_5 },
    ['&']  = { SHIFT, HID_KEY_7 },
    ['\''] = { 0,     HID_KEY_APOSTROPHE },
    ['(']  = { SHIFT, HID_KEY_9 },
    [')']  = { SHIFT, HID_KEY_0 },
    ['*']  = { SHIFT, HID_KEY_8 },
    ['+']  = { SHIFT, HID_KEY_EQUAL },
    [',']  = { 0,     HID_KEY_COMMA },
    ['-']  = { 0,     HID_KEY_MINUS },
    ['.']  = { 0,     HID_KEY_PERIOD },
    ['/']  = { 0,     HID_KEY_SLASH },
    
    // Numbers 0-9
    ['0']  = { 0,     HID_KEY_0 }, ['1'] = { 0, HID_KEY_1 }, ['2'] = { 0, HID_KEY_2 },
    ['3']  = { 0,     HID_KEY_3 }, ['4'] = { 0, HID_KEY_4 }, ['5'] = { 0, HID_KEY_5 },
    ['6']  = { 0,     HID_KEY_6 }, ['7'] = { 0, HID_KEY_7 }, ['8'] = { 0, HID_KEY_8 },
    ['9']  = { 0,     HID_KEY_9 },

    [':']  = { SHIFT, HID_KEY_SEMICOLON },
    [';']  = { 0,     HID_KEY_SEMICOLON },
    ['<']  = { SHIFT, HID_KEY_COMMA },
    ['=']  = { 0,     HID_KEY_EQUAL },
    ['>']  = { SHIFT, HID_KEY_PERIOD },
    ['?']  = { SHIFT, HID_KEY_SLASH },
    ['@']  = { SHIFT, HID_KEY_2 },

    // Uppercase A-Z
    ['A']  = { SHIFT, HID_KEY_A }, ['B'] = { SHIFT, HID_KEY_B }, ['C'] = { SHIFT, HID_KEY_C },
    ['D']  = { SHIFT, HID_KEY_D }, ['E'] = { SHIFT, HID_KEY_E }, ['F'] = { SHIFT, HID_KEY_F },
    ['G']  = { SHIFT, HID_KEY_G }, ['H'] = { SHIFT, HID_KEY_H }, ['I'] = { SHIFT, HID_KEY_I },
    ['J']  = { SHIFT, HID_KEY_J }, ['K'] = { SHIFT, HID_KEY_K }, ['L'] = { SHIFT, HID_KEY_L },
    ['M']  = { SHIFT, HID_KEY_M }, ['N'] = { SHIFT, HID_KEY_N }, ['O'] = { SHIFT, HID_KEY_O },
    ['P']  = { SHIFT, HID_KEY_P }, ['Q'] = { SHIFT, HID_KEY_Q }, ['R'] = { SHIFT, HID_KEY_R },
    ['S']  = { SHIFT, HID_KEY_S }, ['T'] = { SHIFT, HID_KEY_T }, ['U'] = { SHIFT, HID_KEY_U },
    ['V']  = { SHIFT, HID_KEY_V }, ['W'] = { SHIFT, HID_KEY_W }, ['X'] = { SHIFT, HID_KEY_X },
    ['Y']  = { SHIFT, HID_KEY_Y }, ['Z'] = { SHIFT, HID_KEY_Z },

    ['[']  = { 0,     HID_KEY_BRACKET_LEFT },
    ['\\'] = { 0,     HID_KEY_BACKSLASH },
    [']']  = { 0,     HID_KEY_BRACKET_RIGHT },
    ['^']  = { SHIFT, HID_KEY_6 },
    ['_']  = { SHIFT, HID_KEY_MINUS },
    ['`']  = { 0,     HID_KEY_GRAVE },

    // Lowercase a-z
    ['a']  = { 0,     HID_KEY_A }, ['b'] = { 0, HID_KEY_B }, ['c'] = { 0, HID_KEY_C },
    ['d']  = { 0,     HID_KEY_D }, ['e'] = { 0, HID_KEY_E }, ['f'] = { 0, HID_KEY_F },
    ['g']  = { 0,     HID_KEY_G }, ['h'] = { 0, HID_KEY_H }, ['i'] = { 0, HID_KEY_I },
    ['j']  = { 0,     HID_KEY_J }, ['k'] = { 0, HID_KEY_K }, ['l'] = { 0, HID_KEY_L },
    ['m']  = { 0,     HID_KEY_M }, ['n'] = { 0, HID_KEY_N }, ['o'] = { 0, HID_KEY_O },
    ['p']  = { 0,     HID_KEY_P }, ['q'] = { 0, HID_KEY_Q }, ['r'] = { 0, HID_KEY_R },
    ['s']  = { 0,     HID_KEY_S }, ['t'] = { 0, HID_KEY_T }, ['u'] = { 0, HID_KEY_U },
    ['v']  = { 0,     HID_KEY_V }, ['w'] = { 0, HID_KEY_W }, ['x'] = { 0, HID_KEY_X },
    ['y']  = { 0,     HID_KEY_Y }, ['z'] = { 0, HID_KEY_Z },

    ['{']  = { SHIFT, HID_KEY_BRACKET_LEFT },
    ['|']  = { SHIFT, HID_KEY_BACKSLASH },
    ['}']  = { SHIFT, HID_KEY_BRACKET_RIGHT },
    ['~']  = { SHIFT, HID_KEY_GRAVE },
    
    // Support Control Characters
    ['\n'] = { 0,     HID_KEY_ENTER },
    ['\t'] = { 0,     HID_KEY_TAB },
    ['\b'] = { 0,     HID_KEY_BACKSPACE }
};

// Reusable low-level keystroke delivery block
static void app_send_hid_character(uint8_t modifier, uint8_t keycode) {
    uint8_t keycode_arr[6] = { keycode };
    
    // Press Key
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, modifier, keycode_arr);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Release Key
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
    vTaskDelay(pdMS_TO_TICKS(50));
}

// Main API: Iterates through any dynamic C-String and pushes reports
static void app_dynamic_hid_print(const char* str) {
    while (*str) {
        uint8_t ch = (uint8_t)*str;
        
        // Ensure character falls inside safe ASCII structural bounds
        if (ch < 128 && (ascii_hid_table[ch].keycode != 0 || ch == ' ')) {
            app_send_hid_character(ascii_hid_table[ch].modifier, ascii_hid_table[ch].keycode);
        }
        str++;
    }
}

static void app_goto_security_settings() {
    // Press ALT to enter menu bar
    uint8_t keycode_arr[6] = { HID_KEY_ALT_LEFT };
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode_arr);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Release Key
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Go through the menu bar
    app_dynamic_hid_print("CET");

    // Enter Password and press ENTER
    app_dynamic_hid_print("SecretPswd\t\n");
}

static void app_send_mouse_movement(){
    // Mouse output: Move mouse cursor in square trajectory
    ESP_LOGI(TAG, "Sending Mouse report");
    int8_t delta_x;
    int8_t delta_y;
    for (int i = 0; i < (DISTANCE_MAX / DELTA_SCALAR) * 4; i++) {
        // Get the next x and y delta in the draw square pattern
        mouse_draw_square_next_delta(&delta_x, &delta_y);
        tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, delta_x, delta_y, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void app_send_hid_demo(void)
{
    ESP_LOGI(TAG, "Sending Keyboard report");
    app_goto_security_settings();
    // app_dynamic_hid_print("Hello World!!\n");
    // app_send_mouse_movement();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    ESP_LOGI(TAG, "USB device suspended");
    suspended = true;
    if (remote_wakeup_en) {
        ESP_LOGI(TAG, "Remote wakeup available, press the button to wake up the Host");
        wakeup_host = true;
    } else {
        ESP_LOGI(TAG, "Remote wakeup not available");
    }
}

void tud_resume_cb(void)
{
    ESP_LOGI(TAG, "USB device resumed");
    suspended = false;
}

void app_main(void)
{
    // Initialize button that will trigger HID reports
    const gpio_config_t boot_button_config = {
        .pin_bit_mask = BIT64(APP_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_button_config));

    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();

    tusb_cfg.descriptor.device = NULL;
    tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
    tusb_cfg.descriptor.string = hid_string_descriptor;
    tusb_cfg.descriptor.string_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]);
#if (TUD_OPT_HIGH_SPEED)
    tusb_cfg.descriptor.high_speed_config = hid_configuration_descriptor;
#endif // TUD_OPT_HIGH_SPEED

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    while (1) {
        if (tud_mounted()) {
            static bool send_hid_data = false;
            if (send_hid_data) {
                if (!suspended) {
                    app_send_hid_demo();
                } else {
                    if (wakeup_host) {
                        ESP_LOGI(TAG, "Waking up the Host");
                        tud_remote_wakeup();
                        wakeup_host = false;
                    } else {
                        ESP_LOGI(TAG, "USB Host remote wakeup is not available.");
                    }
                }
            }
            send_hid_data = !gpio_get_level(APP_BUTTON);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
