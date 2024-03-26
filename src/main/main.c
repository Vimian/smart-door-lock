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

bool isLocked = true;
bool isOpened = false;
bool isAlarm = false;

bool locking = false;
bool unlocking = false;
bool opening = false;
bool closing = false;
int unlockTimer = 0;

bool openButton = false;
bool lockButton = false;

void initial() {
    if (isLocked && !isOpened) {
        state = LOCKED;
    } else if (!isLocked && !isOpened) {
        state = UNLOCKED;
    } else if (isOpened) {
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
    if (isLocked) {
        isAlarm = true;
        state = ALARM;
    } else if (!isLocked) {
        state = UNLOCKED;
    }
}

void funlocking() {
    if (unlockTimer == 10 && !isOpened) {
        state = LOCKED;
    } else if (unlockTimer < 10 && !isOpened) {
        isAlarm = false;
        isLocked = false;
        state = UNLOCKED;
    } else if (isOpened) {
        isAlarm = false;
        isLocked = false;
        state = OPENED;
    } else if (unlockTimer == 10 && isAlarm) {
        state = ALARM;
    } else if (unlockTimer == 10 && isOpened && !isAlarm) {
        state = AUTOLOCK;
    }

    unlockTimer++;
}

void falarm() {
    if (unlocking) {
        state = UNLOCKING;
    }
}

void autoLock() {
    if (unlocking) {
        state = UNLOCKING;
    } else if (closing && !isOpened) {
        state = LOCKED;
    }
}

void TaskBlink (void *pvParameters) {
    uint8_t alarm_state = 1;

    while (1) {
        //printf("Changing led state\n");

        gpio_set_level(PIN_OPENED, isOpened ? 0 : 1);
        gpio_set_level(PIN_LOCKED, isLocked ? 0 : 1);
        gpio_set_level(PIN_ALARM, alarm_state);
        vTaskDelay(DELAY);

        if (isAlarm || (!isAlarm && alarm_state == 0)) {
            alarm_state = alarm_state == 0 ? 1 : 0;
        }
    }
}

void ListenToButtons (void *pvParameters) {
    while (1) {
        //printf("Listening to buttons\n");
        
        if (gpio_get_level(PIN_LOCK) == 0 && !lockButton) {
            lockButton = true;

            if (isLocked) {
                unlocking = true;
                unlockTimer = 0;
            } else {
                locking = true;
                isLocked = true;
            }
        } else if (gpio_get_level(PIN_LOCK) == 1 && lockButton) {
            lockButton = false;
        }

        if (gpio_get_level(PIN_OPEN) == 0 && !openButton) {
            openButton = true;

            if (isOpened) {
                closing = true;
                isOpened = false;
            } else {
                opening = true;
                isOpened = true;
            }
        } else if (gpio_get_level(PIN_OPEN) == 1 && openButton) {
            openButton = false;
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

    xTaskCreate(TaskBlink, "Blink", 4096, NULL, 1, NULL);

    xTaskCreate(ListenToButtons, "Buttons", 4096, NULL, 1, NULL);

    while (1) {
        printf("{ State is: %d, isOpened: %d, isLocked: %d, isAlarm: %d }\n", state, isOpened, isLocked, isAlarm);

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
                funlocking();
                break;
            case ALARM:
                falarm();
                break;
            case AUTOLOCK:
                locking = false;
                autoLock();
                break;
            default:
                printf("Error state not defined!");
                // TODO: Add error handling
                break;
        }
        sleep(1);
    }
}
