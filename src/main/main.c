#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/gpio.h"
#include "unistd.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "string.h"

#define PIN_OPENED (15)
#define PIN_LOCKED (2)
#define PIN_ALARM (4)
#define PIN_OPEN GPIO_NUM_18
#define PIN_LOCK GPIO_NUM_19

#define DELAY (500 / portTICK_PERIOD_MS)

enum States {
    INITIAL, // 0
    LOCKED, // 1
    UNLOCKED, // 2
    OPENED, // 3
    UNLOCKING, // 4
    ALARM, // 5
    AUTOLOCK // 6
};

enum States state = INITIAL;

bool is_locked = true;
bool is_opened = false;
bool is_alarm = false;

bool locking = false;
bool unlocking = false;
bool opening = false;
bool closing = false;
int unlock_timer = 0;

bool open_button = false;
bool lock_button = false;
bool bt_connected = false;

void initial() {
    if (is_locked && !is_opened) {
        state = LOCKED;
    } else if (!is_locked && !is_opened) {
        state = UNLOCKED;
    } else if (is_opened) {
        state = OPENED;
    }
}

void locked() {
    if (opening) {
        state = OPENED;
    } else if (unlocking) {
        state = UNLOCKING;
    }
}

void unlocked() {
    if (locking) {
        state = LOCKED;
    } else if (opening) {
        state = OPENED;
    }
}

void opened() {
    if (is_locked) {
        is_alarm = true;
        state = ALARM;
    } else if (!is_locked) {
        state = UNLOCKED;
    }
}

void func_unlocking() {
    if (unlock_timer == 10 && !is_opened) {
        state = LOCKED;
    } else if (unlock_timer < 10 && !is_opened) {
        is_alarm = false;
        is_locked = false;
        state = UNLOCKED;
    } else if (is_opened) {
        is_alarm = false;
        is_locked = false;
        state = OPENED;
    } else if (unlock_timer == 10 && is_alarm) {
        state = ALARM;
    } else if (unlock_timer == 10 && is_opened && !is_alarm) {
        state = AUTOLOCK;
    }

    unlock_timer++;
}

void func_alarm() {
    if (unlocking) {
        state = UNLOCKING;
    }
}

void auto_lock() {
    if (unlocking) {
        state = UNLOCKING;
    } else if (closing && !is_opened) {
        state = LOCKED;
    }
}

void task_blink (void *pvParameters) {
    uint8_t alarm_state = 1;

    while (1) {
        //printf("Changing led state\n");

        gpio_set_level(PIN_OPENED, is_opened ? 0 : 1);
        gpio_set_level(PIN_LOCKED, is_locked ? 0 : 1);
        gpio_set_level(PIN_ALARM, alarm_state);
        vTaskDelay(DELAY);

        if (is_alarm || alarm_state == 0) {
            alarm_state = alarm_state == 0 ? 1 : 0;
        }
    }
}

void listen_to_buttons (void *pvParameters) {
    while (1) {
        //printf("Listening to buttons\n");
        
        if (gpio_get_level(PIN_LOCK) == 0 && !lock_button) {
            lock_button = true;

            if (is_locked) {
                unlocking = true;
                unlock_timer = 0;
            } else {
                locking = true;
                is_locked = true;
            }
        } else if (gpio_get_level(PIN_LOCK) == 1 && lock_button) {
            lock_button = false;
        }

        if (gpio_get_level(PIN_OPEN) == 0 && !open_button) {
            open_button = true;

            if (is_opened) {
                closing = true;
                is_opened = false;
            } else {
                opening = true;
                is_opened = true;
            }
        } else if (gpio_get_level(PIN_OPEN) == 1 && open_button) {
            open_button = false;
        }

        vTaskDelay(10);
    }
}

esp_bd_addr_t trusted_device;
bool paired = false;

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size) {
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return str;
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    char bda_str[18] = {0};
    
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            //printf("Authentication status: %d, name: %s bda:[%s]", param->auth_cmpl.stat, param->auth_cmpl.device_name, bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                memcpy(trusted_device, param->auth_cmpl.bda, sizeof(esp_bd_addr_t));
            }
            break;
        }
        case ESP_BT_GAP_PIN_REQ_EVT: {
            printf("ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
            if (param->pin_req.min_16_digit) {
                printf("Input pin code: 0000 0000 0000 0000");
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                printf("Input pin code: 1234");
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '1';
                pin_code[1] = '2';
                pin_code[2] = '3';
                pin_code[3] = '4';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }
        case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT: {
            //printf("Device connection status: %d\n", param->acl_conn_cmpl_stat.stat);
            if (param->acl_conn_cmpl_stat.stat != ESP_BT_STATUS_HCI_SUCCESS || !paired) {
                break;
            }

            bt_connected = true;

            if (is_locked && !unlocking) {
                unlocking = true;
                unlock_timer = 0;
            }
            break;
        }
        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT: {
            //printf("Device disconnected status: %d\n", param->acl_disconn_cmpl_stat.reason);
            paired = true;
            bt_connected = false;

            if (!is_locked && !locking){
                locking = true;
                is_locked = true;
            }
            break;
        }
        default: {
            //printf("Unhandled event: %d\n", event);
        }
    }
}

void ping_device(void *pvParameters) {
    while (1) {
        if (!paired) {
            vTaskDelay(1000);
            continue;
        }
        esp_bt_gap_get_remote_services(trusted_device);
        vTaskDelay(100);
    }
}

void setup_bt() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        printf("Failed to initialize controller\n");
        return;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
        printf("Failed to enable controller\n");
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en = false;

    if (esp_bluedroid_init_with_cfg(&bluedroid_cfg) != ESP_OK) {
        printf("Failed to initialize bluedroid\n");
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        printf("Failed to enable bluedroid\n");
        return;
    }

    if (esp_bt_gap_register_callback(esp_bt_gap_cb) != ESP_OK) {
        printf("Failed to register GAP callback\n");
        return;
    }

    esp_bt_dev_set_device_name("SmartDoorLock");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
}

void app_main(void) {
    gpio_reset_pin(PIN_OPENED);
    gpio_set_direction(PIN_OPENED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_LOCKED);
    gpio_set_direction(PIN_LOCKED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_ALARM);
    gpio_set_direction(PIN_ALARM, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_OPEN);
    gpio_set_direction(PIN_OPEN, GPIO_MODE_INPUT);
    gpio_reset_pin(PIN_LOCK);
    gpio_set_direction(PIN_LOCK, GPIO_MODE_INPUT);

    setup_bt();

    xTaskCreate(task_blink, "Blink", 4096, NULL, 1, NULL);

    xTaskCreate(listen_to_buttons, "Buttons", 4096, NULL, 1, NULL);
    
    xTaskCreate(ping_device, "PingDevice", 4096, NULL, 1, NULL);

    while (1) {
        esp_bluedroid_status_t status = esp_bluedroid_get_status();
        char bda_str[18] = {0};
        printf("{ State is: %d, is_opened: %d, is_locked: %d, is_alarm: %d, bt_connected: %d, bt_status: %d, ESP32_bt_addr: %s }\n", state, is_opened, is_locked, is_alarm, bt_connected, status, bda2str(esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

        switch (state) {
            case INITIAL:
                initial();
                break;
            case LOCKED:
                locking = false;
                closing = false;
                locked();
                break;
            case UNLOCKED:
                closing = false;
                unlocked();
                break;
            case OPENED:
                opening = false;
                opened();
                break;
            case UNLOCKING:
                unlocking = false;
                func_unlocking();
                break;
            case ALARM:
                func_alarm();
                break;
            case AUTOLOCK:
                locking = false;
                auto_lock();
                break;
            default:
                printf("Error state not defined!");
                // TODO: Add error handling
                break;
        }
        sleep(1);
    }
}