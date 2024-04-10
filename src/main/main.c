#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/gpio.h"
#include "unistd.h"

#define PIN_OPENED (15)
#define PIN_LOCKED (2)
#define PIN_ALARM (4)
#define PIN_OPEN GPIO_NUM_32
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

void listen_to_analog (void *pvParameters) {
    while (1) {
        //printf("Listening to analog\n");
        int value = GPIO_NUM_32;
        if (gpio_get_level(PIN_OPEN) == 1 && value <= 2000)
        {
            closing = true;
            is_opened = false;
        } else {
            opening = true;
            is_opened = true;
        }
        

        vTaskDelay(10);
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

        vTaskDelay(10);
    }
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

    xTaskCreate(task_blink, "Blink", 4096, NULL, 1, NULL);

    xTaskCreate(listen_to_buttons, "Buttons", 4096, NULL, 1, NULL);

    xTaskCreate(listen_to_analog, "Analog", 4096, NULL, 1, NULL);

    while (1) {
        printf("{ State is: %d, is_opened: %d, is_locked: %d, is_alarm: %d }\n", state, is_opened, is_locked, is_alarm);

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
