#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/gpio.h"
#include "unistd.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "string.h"

#define PIN_OPENED (15)

#define DELAY (500 / portTICK_PERIOD_MS)

bool locking = false;
bool unlocking = false;

bool connected = false;

void TaskBlink (void *pvParameters) {
    while (1) {
        gpio_set_level(PIN_OPENED, connected ? 0 : 1);
        vTaskDelay(DELAY);
    }
}

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size) {
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return str;
}

esp_bd_addr_t trusted_device;
bool paired = false;

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    char bda_str[18] = {0};
    
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                printf("authentication success: %s bda:[%s]", param->auth_cmpl.device_name, bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
                memcpy(trusted_device, param->auth_cmpl.bda, sizeof(esp_bd_addr_t));
            } else {
                printf("authentication failed, status:%d", param->auth_cmpl.stat);
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
            if (!paired) {
                break;
            }
            if (param->acl_conn_cmpl_stat.stat != ESP_BT_STATUS_HCI_SUCCESS) {
                printf("Device connection failed: %d\n", param->acl_conn_cmpl_stat.stat);
                break;
            }
            printf("Device connected\n");
            connected = true;
            break;
        }
        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT: {
            paired = true;
            printf("Device disconnected status: %d\n", param->acl_disconn_cmpl_stat.reason);
            connected = false;
            break;
        }
        default: {
            printf("Unhandled event: %d\n", event);
        }
    }
}

void PingDevice(void *pvParameters) {
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
    bluedroid_cfg.ssp_en = true;

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

    setup_bt();

    xTaskCreate(TaskBlink, "Blink", 4096, NULL, 1, NULL);

    xTaskCreate(PingDevice, "PingDevice", 4096, NULL, 1, NULL);

    while (1) {
        esp_bluedroid_status_t status = esp_bluedroid_get_status();
        printf("{ locking: %d, unlocking: %d, connected: %d, bt_status: %d }\n", locking, unlocking, connected, status);
        char bda_str[18] = {0};
        printf("Address: %s\n", bda2str(esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

        /*printf("%d\n", ESP_BT_STATUS_SUCCESS);
        printf("%d\n", ESP_BT_STATUS_FAIL);
        printf("%d\n", ESP_BT_STATUS_NOT_READY);
        printf("%d\n", ESP_BT_STATUS_NOMEM);
        printf("%d\n", ESP_BT_STATUS_BUSY);
        printf("%d\n", ESP_BT_STATUS_DONE);
        printf("%d\n", ESP_BT_STATUS_UNSUPPORTED);
        printf("%d\n", ESP_BT_STATUS_PARM_INVALID);
        printf("%d\n", ESP_BT_STATUS_UNHANDLED);
        printf("%d\n", ESP_BT_STATUS_AUTH_FAILURE);
        printf("%d\n", ESP_BT_STATUS_RMT_DEV_DOWN);
        printf("%d\n", ESP_BT_STATUS_AUTH_REJECTED);
        printf("%d\n", ESP_BT_STATUS_INVALID_STATIC_RAND_ADDR);
        printf("%d\n", ESP_BT_STATUS_PENDING);
        printf("%d\n", ESP_BT_STATUS_UNACCEPT_CONN_INTERVAL);
        printf("%d\n", ESP_BT_STATUS_PARAM_OUT_OF_RANGE);
        printf("%d\n", ESP_BT_STATUS_TIMEOUT);
        printf("%d\n", ESP_BT_STATUS_PEER_LE_DATA_LEN_UNSUPPORTED);
        printf("%d\n", ESP_BT_STATUS_CONTROL_LE_DATA_LEN_UNSUPPORTED);
        printf("%d\n", ESP_BT_STATUS_ERR_ILLEGAL_PARAMETER_FMT);
        printf("%d\n", ESP_BT_STATUS_MEMORY_FULL);
        printf("%d\n", ESP_BT_STATUS_EIR_TOO_LARGE);
        printf("%d\n", ESP_BT_STATUS_HCI_SUCCESS);
        printf("%d\n", ESP_BT_STATUS_HCI_ILLEGAL_COMMAND);
        printf("%d\n", ESP_BT_STATUS_HCI_NO_CONNECTION);
        printf("%d\n", ESP_BT_STATUS_HCI_HW_FAILURE);
        printf("%d\n", ESP_BT_STATUS_HCI_PAGE_TIMEOUT);
        printf("%d\n", ESP_BT_STATUS_HCI_AUTH_FAILURE);
        printf("%d\n", ESP_BT_STATUS_HCI_KEY_MISSING);
        printf("%d\n", ESP_BT_STATUS_HCI_MEMORY_FULL);
        printf("%d\n", ESP_BT_STATUS_HCI_CONNECTION_TOUT);
        printf("%d\n", ESP_BT_STATUS_HCI_MAX_NUM_OF_CONNECTIONS);
        printf("%d\n", ESP_BT_STATUS_HCI_MAX_NUM_OF_SCOS);
        printf("%d\n", ESP_BT_STATUS_HCI_CONNECTION_EXISTS);
        printf("%d\n", ESP_BT_STATUS_HCI_COMMAND_DISALLOWED);
        printf("%d\n", ESP_BT_STATUS_HCI_HOST_REJECT_RESOURCES);
        printf("%d\n", ESP_BT_STATUS_HCI_HOST_REJECT_SECURITY);
        printf("%d\n", ESP_BT_STATUS_HCI_HOST_REJECT_DEVICE);
        printf("%d\n", ESP_BT_STATUS_HCI_HOST_TIMEOUT);
        printf("%d\n", ESP_BT_STATUS_HCI_UNSUPPORTED_VALUE);
        printf("%d\n", ESP_BT_STATUS_HCI_ILLEGAL_PARAMETER_FMT);
        printf("%d\n", ESP_BT_STATUS_HCI_PEER_USER);
        printf("%d\n", ESP_BT_STATUS_HCI_PEER_LOW_RESOURCES);
        printf("%d\n", ESP_BT_STATUS_HCI_PEER_POWER_OFF);
        printf("%d\n", ESP_BT_STATUS_HCI_CONN_CAUSE_LOCAL_HOST);
        printf("%d\n", ESP_BT_STATUS_HCI_REPEATED_ATTEMPTS);
        printf("%d\n", ESP_BT_STATUS_HCI_PAIRING_NOT_ALLOWED);
        printf("%d\n", ESP_BT_STATUS_HCI_UNKNOWN_LMP_PDU);
        printf("%d\n", ESP_BT_STATUS_HCI_UNSUPPORTED_REM_FEATURE);
        printf("%d\n", ESP_BT_STATUS_HCI_SCO_OFFSET_REJECTED);
        printf("%d\n", ESP_BT_STATUS_HCI_SCO_INTERVAL_REJECTED);
        printf("%d\n", ESP_BT_STATUS_HCI_SCO_AIR_MODE);
        printf("%d\n", ESP_BT_STATUS_HCI_INVALID_LMP_PARAM);
        printf("%d\n", ESP_BT_STATUS_HCI_UNSPECIFIED);
        printf("%d\n", ESP_BT_STATUS_HCI_UNSUPPORTED_LMP_PARAMETERS);
        printf("%d\n", ESP_BT_STATUS_HCI_ROLE_CHANGE_NOT_ALLOWED);
        printf("%d\n", ESP_BT_STATUS_HCI_LMP_RESPONSE_TIMEOUT);
        printf("%d\n", ESP_BT_STATUS_HCI_LMP_ERR_TRANS_COLLISION);
        printf("%d\n", ESP_BT_STATUS_HCI_LMP_PDU_NOT_ALLOWED);
        printf("%d\n", ESP_BT_STATUS_HCI_ENCRY_MODE_NOT_ACCEPTABLE);
        printf("%d\n", ESP_BT_STATUS_HCI_UNIT_KEY_USED);
        printf("%d\n", ESP_BT_STATUS_HCI_QOS_NOT_SUPPORTED);
        printf("%d\n", ESP_BT_STATUS_HCI_INSTANT_PASSED);
        printf("%d\n", ESP_BT_STATUS_HCI_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED);
        printf("%d\n", ESP_BT_STATUS_HCI_DIFF_TRANSACTION_COLLISION);
        printf("%d\n", ESP_BT_STATUS_HCI_UNDEFINED_0x2B);
        printf("%d\n", ESP_BT_STATUS_HCI_QOS_UNACCEPTABLE_PARAM);
        printf("%d\n", ESP_BT_STATUS_HCI_QOS_REJECTED);
        printf("%d\n", ESP_BT_STATUS_HCI_CHAN_CLASSIF_NOT_SUPPORTED);
        printf("%d\n", ESP_BT_STATUS_HCI_INSUFFCIENT_SECURITY);
        printf("%d\n", ESP_BT_STATUS_HCI_PARAM_OUT_OF_RANGE);
        printf("%d\n", ESP_BT_STATUS_HCI_UNDEFINED_0x31);
        printf("%d\n", ESP_BT_STATUS_HCI_ROLE_SWITCH_PENDING);
        printf("%d\n", ESP_BT_STATUS_HCI_UNDEFINED_0x33);
        printf("%d\n", ESP_BT_STATUS_HCI_RESERVED_SLOT_VIOLATION);
        printf("%d\n", ESP_BT_STATUS_HCI_ROLE_SWITCH_FAILED);
        printf("%d\n", ESP_BT_STATUS_HCI_INQ_RSP_DATA_TOO_LARGE);
        printf("%d\n", ESP_BT_STATUS_HCI_SIMPLE_PAIRING_NOT_SUPPORTED);
        printf("%d\n", ESP_BT_STATUS_HCI_HOST_BUSY_PAIRING);
        printf("%d\n", ESP_BT_STATUS_HCI_REJ_NO_SUITABLE_CHANNEL);
        printf("%d\n", ESP_BT_STATUS_HCI_CONTROLLER_BUSY);
        printf("%d\n", ESP_BT_STATUS_HCI_UNACCEPT_CONN_INTERVAL);
        printf("%d\n", ESP_BT_STATUS_HCI_DIRECTED_ADVERTISING_TIMEOUT);
        printf("%d\n", ESP_BT_STATUS_HCI_CONN_TOUT_DUE_TO_MIC_FAILURE);
        printf("%d\n", ESP_BT_STATUS_HCI_CONN_FAILED_ESTABLISHMENT);
        printf("%d\n", ESP_BT_STATUS_HCI_MAC_CONNECTION_FAILED);
        */

        /*if (paired) {
            esp_bt_gap_get_remote_services(trusted_device);
        }*/

        sleep(1);
    }
}
