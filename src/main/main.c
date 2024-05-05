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

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

#define PIN_OPENED (15)
#define PIN_LOCKED (2)
#define PIN_ALARM (4)
#define PIN_OPEN GPIO_NUM_18
#define PIN_LOCK GPIO_NUM_19

#define DELAY (500 / portTICK_PERIOD_MS)
#define DEBUG_ON  0

#if DEBUG_ON
#define EXAMPLE_DEBUG ESP_LOGI
#else
#define EXAMPLE_DEBUG( tag, format, ... )
#endif

#define BLE_TAG "SMART_DOORLOCK"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "SMART_DOORLOCK"
#define SVC_INST_ID                 0

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)


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

static uint8_t adv_config_done       = 0;

static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 16,
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x40,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};


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

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

esp_bd_addr_t trusted_device;
bool paired = false;


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(BLE_TAG, "(0) ***** advertising start successfully ***** \n");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(BLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_NC_REQ_EVT:
            /* The app will receive this event when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
            show the passkey number to the user to confirm it with the number displayed by peer device. */
            ESP_LOGI(BLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%" PRIu32, param->ble_security.key_notif.passkey);
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            /* send the positive(true) security response to the peer device to accept the security request.
            If not accept the security request, should send the security response with negative(false) accept value*/
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO has Output capability and the peer device IO has Input capability.
            ///show the passkey number to the user to input it in the peer device.
            ESP_LOGI(BLE_TAG, "The passkey notify number:%06" PRIu32, param->ble_security.key_notif.passkey);
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT: {
            esp_bd_addr_t bd_addr;
            memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
            EXAMPLE_DEBUG(BLE_TAG, "remote BD_ADDR: %08x%04x",\
                    (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                    (bd_addr[4] << 8) + bd_addr[5]);
            EXAMPLE_DEBUG(BLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
            if (param->ble_security.auth_cmpl.success){
                ESP_LOGI(BLE_TAG, "(1) ***** pair status = success ***** \n");
                printf("pair mac address: %02x:%02x:%02x:%02x:%02x:%02x\n", bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
                paired = true;
                gatts_profile_event_handler(ESP_GATTS_CONNECT_EVT, heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if, param);
            }
            else {
                ESP_LOGI(BLE_TAG, "***** pair status = fail, reason = 0x%x *****\n", param->ble_security.auth_cmpl.fail_reason);
            }
            break;
        }
        default:
            //printf("gap unknown event %d\n", event); //check for unknown events
            break;
    }
}


static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            if (esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME) != ESP_OK){
                printf("set device name failed, error code \n");
            }
    
            //config adv data
            if (esp_ble_gap_config_adv_data(&adv_data) != ESP_OK){
                printf("config adv data failed, error code \n");
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            if (esp_ble_gap_config_adv_data(&scan_rsp_data) != ESP_OK){
                printf("config scan response data failed, error code \n");
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        }
       	    break;
        case ESP_GATTS_START_EVT:
            EXAMPLE_DEBUG(BLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(BLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            /* start security connect with peer device when receive the connect event sent by the master */
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            memcpy(trusted_device, param->connect.remote_bda, sizeof(esp_bd_addr_t));

            if (paired == true) {

                bt_connected = true;

                if (is_locked && !unlocking) {
                    unlocking = true;
                    unlock_timer = 0;
            
                }
            }
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = %d", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            paired = false; 

            bt_connected = false;

            if (!is_locked && !locking){
                locking = true;
                is_locked = true;
            }
            break;
        default:
            //printf("gatt unknown event %d\n", event); //check for unknown events
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(BLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void setup_bt() {
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        printf( "initialize controller failed\n");
        return;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK){ 
        printf( "enable bluetooth failed\n");
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en = false;
    if (esp_bluedroid_init_with_cfg(&bluedroid_cfg) != ESP_OK){
        printf( "init bluetooth failed\n");
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        printf( "enable bluetooth failed\n");
        return;
    }
    if (esp_ble_gatts_register_callback(gatts_event_handler) != ESP_OK){
        printf( "gatts register failed\n");
        return;
    }
    if (esp_ble_gap_register_callback(gap_event_handler) != ESP_OK){
        printf( "gap register failed\n");
        return;
    }
    if (esp_ble_gatts_app_register(ESP_APP_ID) != ESP_OK){
        printf( "gatts app register failed\n");
        return;
    }
    if (esp_ble_gatt_set_local_mtu(33) != ESP_OK){
        printf( "set local  MTU failed\n");
        return;
    }

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint32_t passkey = 123456;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

void app_main(void)
{
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
    
    while (1) {
        esp_bluedroid_status_t status = esp_bluedroid_get_status();
        printf("{ State is: %d, is_opened: %d, is_locked: %d, is_alarm: %d, bt_connected: %d, bt_status: %d }\n", state, is_opened, is_locked, is_alarm, bt_connected, status);

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
