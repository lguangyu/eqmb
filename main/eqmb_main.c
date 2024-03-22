/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define TAG "EQMB"

// these variables should only be read/set in critical sections
static SemaphoreHandle_t eqmb_ble_semaphore = NULL;
// pairing status
static bool eqmb_pairing_on = false;
static QueueHandle_t eqmb_pairing_gpio_queue = NULL;
static TimerHandle_t eqmb_pairing_timer = NULL;
static int64_t eqmb_pairing_gpio_last_act_time;
// bluetooth status
static bool eqmb_connected = false; // allow only one connection
static esp_bd_addr_t eqmb_current_remote_addr = {0};
const static esp_bd_addr_t eqmb_static_local_addr = {0xd6, 0x3c, 0x1e, 0x0b, 0x73, 0x15};

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

const static gpio_num_t eqmb_ragebtn_gpio_num = GPIO_NUM_18;	 // ? button for april to rage-press
const static gpio_num_t eqmb_pairing_led_gpio_num = GPIO_NUM_19; // led to indicate advertising
const static gpio_num_t eqmb_pairing_gpio_num = GPIO_NUM_21;	 // button to start pairing

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME "HID"
static uint8_t hidd_service_uuid128[] = {
	0xfb,
	0x34,
	0x9b,
	0x5f,
	0x80,
	0x00,
	0x00,
	0x80,
	0x00,
	0x10,
	0x00,
	0x00,
	0x12,
	0x18,
	0x00,
	0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
	.set_scan_rsp = false,
	.include_name = true,
	.include_txpower = false,
	.min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
	.max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
	.appearance = 0x03c1,	// HID Keyboard,
	.manufacturer_len = 0,
	.p_manufacturer_data = NULL,
	.service_data_len = 0,
	.p_service_data = NULL,
	.service_uuid_len = sizeof(hidd_service_uuid128),
	.p_service_uuid = hidd_service_uuid128,
	.flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
	.adv_int_min = 0x20,
	.adv_int_max = 0x30,
	.adv_type = ADV_TYPE_IND,
	.own_addr_type = BLE_ADDR_TYPE_RANDOM,
	//.peer_addr            =
	//.peer_addr_type       =
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST,
};

static void eqmb_pairing_led_task(void *arg)
{
	bool led_on = false;
	while (1)
	{
		led_on = !led_on;
		gpio_set_level(GPIO_NUM_2, led_on && eqmb_pairing_on);
		gpio_set_level(eqmb_pairing_led_gpio_num, led_on && eqmb_pairing_on);
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}
	return;
}

static void eqmb_pairing_start(void)
{
	xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
	if (!eqmb_pairing_on)
	{
		eqmb_pairing_on = true;
		hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
		esp_ble_gap_stop_advertising();
		if (!eqmb_connected)
			esp_ble_gap_start_advertising(&hidd_adv_params);
		xTimerStart(eqmb_pairing_timer, 0);
	}
	xSemaphoreGive(eqmb_ble_semaphore);
	return;
}

static void eqmb_pairing_stop(void)
{
	xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
	if (eqmb_pairing_on)
	{
		eqmb_pairing_on = false;
		hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST;
		esp_ble_gap_stop_advertising();
		if (!eqmb_connected)
			esp_ble_gap_start_advertising(&hidd_adv_params);
		xSemaphoreGive(eqmb_ble_semaphore);
	}
	return;
}

static void eqmb_pairing_timer_callback(TimerHandle_t xTimer)
{
	eqmb_pairing_stop();
	return;
}

static void eqmb_pairing_task(void *arg)
{
	while (1)
	{
		if (xQueueReceive(eqmb_pairing_gpio_queue, NULL, portMAX_DELAY))
		{
			if (eqmb_connected)
				esp_ble_gap_disconnect(eqmb_current_remote_addr);
			if (!eqmb_pairing_on)
			{
				ESP_LOGI(TAG, "Start pairing mode");
				eqmb_pairing_start();
			}
			else
			{
				ESP_LOGI(TAG, "Stop pairing mode");
				eqmb_pairing_stop();
			}
		}
	}
	return;
}

static void IRAM_ATTR eqmb_pairing_gpio_isr(void *arg)
{
	// with jitter protection
	int64_t now = esp_timer_get_time();
	if (now < eqmb_pairing_gpio_last_act_time + 200000) // 200ms
		return;
	xQueueSendFromISR(eqmb_pairing_gpio_queue, NULL, NULL);
	eqmb_pairing_gpio_last_act_time = now;
	return;
}

static void eqmb_adv_gpio_init(void)
{
	// leds
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(eqmb_pairing_led_gpio_num, GPIO_MODE_OUTPUT);
	// create tasks
	eqmb_pairing_gpio_queue = xQueueCreate(1, 0);
	if (eqmb_pairing_gpio_queue == NULL)
	{
		ESP_LOGE(TAG, "eqmb_pairing_gpio_queue create = fail");
		return;
	}
	eqmb_pairing_timer = xTimerCreate("eqmb_pairing_timer",
									  30000 / portTICK_PERIOD_MS,
									  pdFALSE, NULL,
									  eqmb_pairing_timer_callback);
	if (eqmb_pairing_timer == NULL)
	{
		ESP_LOGE(TAG, "eqmb_pairing_timer create = fail");
		return;
	}
	xTaskCreate(&eqmb_pairing_task, "eqmb_pairing_task", 2048, NULL, 5, NULL);
	xTaskCreate(&eqmb_pairing_led_task, "eqmb_pairing_led_task", 1024, NULL, 5, NULL);
	// connect pin
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = 1ULL << eqmb_pairing_gpio_num;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(eqmb_pairing_gpio_num, eqmb_pairing_gpio_isr, NULL);
	// pin jitter protection
	eqmb_pairing_gpio_last_act_time = esp_timer_get_time();
	return;
}

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
	switch (event)
	{
	case ESP_HIDD_EVENT_REG_FINISH:
		if (param->init_finish.state == ESP_HIDD_INIT_OK)
		{
			esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
			esp_ble_gap_config_adv_data(&hidd_adv_data);
		}
		break;
	case ESP_BAT_EVENT_REG:
		break;
	case ESP_HIDD_EVENT_DEINIT_FINISH:
		break;
	case ESP_HIDD_EVENT_BLE_CONNECT:
		ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
		eqmb_connected = true;
		esp_ble_gap_stop_advertising();
		break;
	case ESP_HIDD_EVENT_BLE_DISCONNECT:
		ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
		eqmb_connected = false;
		esp_ble_gap_start_advertising(&hidd_adv_params);
		break;
	case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT:
		ESP_LOGI(TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
		ESP_LOG_BUFFER_HEX(TAG, param->vendor_write.data, param->vendor_write.length);
		break;
	case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT:
		ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
		ESP_LOG_BUFFER_HEX(TAG, param->led_write.data, param->led_write.length);
		break;
	default:
		break;
	}
	return;
}

static bool is_device_bound(esp_bd_addr_t bd_addr)
{
	int bond_dev_num = esp_ble_get_bond_device_num();
	esp_ble_bond_dev_t *bond_dev_list = (esp_ble_bond_dev_t *)malloc(bond_dev_num * sizeof(esp_ble_bond_dev_t));
	esp_ble_get_bond_device_list(&bond_dev_num, bond_dev_list);
	bool ret = false;
	for (int i = 0; i < bond_dev_num; i++)
		if (memcmp(bond_dev_list[i].bd_addr, bd_addr, sizeof(esp_bd_addr_t)) == 0)
		{
			ret = true;
			break;
		}
	free(bond_dev_list);
	return ret;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event)
	{
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		if (!eqmb_connected)
			esp_ble_gap_start_advertising(&hidd_adv_params);
		break;
	case ESP_GAP_BLE_SEC_REQ_EVT:
		for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
			ESP_LOGD(TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		break;
	case ESP_GAP_BLE_AUTH_CMPL_EVT:
		esp_bd_addr_t bd_addr;
		memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
		ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x",
				 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
				 (bd_addr[4] << 8) + bd_addr[5]);
		ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
		ESP_LOGI(TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
		if (param->ble_security.auth_cmpl.success)
		{
			// connected, update connection status
			xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
			memcpy(eqmb_current_remote_addr, bd_addr, sizeof(esp_bd_addr_t));
			xSemaphoreGive(eqmb_ble_semaphore);
			// heuristically stop pairing mode if new device found
			if (!is_device_bound(bd_addr))
			{
				eqmb_pairing_stop();
				esp_ble_gap_update_whitelist(true, bd_addr, BLE_ADDR_TYPE_PUBLIC);
			}
		}
		else
			ESP_LOGE(TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
		break;
	default:
		break;
	}
}

static void eqmb_nvs_flash_init(void)
{
	esp_err_t ret;
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	return;
}

static void eqmb_bt_controller_init(void)
{
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
	ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
	return;
}

static void eqmb_bluedroid_init(void)
{
	esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));
	ESP_ERROR_CHECK(esp_bluedroid_enable());
	return;
}

static void eqmb_gap_init(void)
{
	/* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
	esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
	esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;		// set the IO capability to No output No input
	uint8_t key_size = 16;							// the key size should be 7~16 bytes
	uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
	/* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
	and the response key means which key you can distribute to the Master;
	If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
	and the init key means which key you can distribute to the slave. */
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
	return;
}

static void eqmb_load_bond_devices_as_whitelist(void)
{
	int bond_dev_num = esp_ble_get_bond_device_num();
	esp_ble_bond_dev_t *bond_dev_list = (esp_ble_bond_dev_t *)malloc(bond_dev_num * sizeof(esp_ble_bond_dev_t));
	esp_ble_get_bond_device_list(&bond_dev_num, bond_dev_list);
	for (int i = 0; i < bond_dev_num; i++)
	{
		ESP_ERROR_CHECK(esp_ble_gap_update_whitelist(true, bond_dev_list[i].bd_addr, BLE_ADDR_TYPE_PUBLIC));
		ESP_LOGI(TAG, "Adding bond device to whitelist: %08x%04x",
				 (bond_dev_list[i].bd_addr[0] << 24) + (bond_dev_list[i].bd_addr[1] << 16) + (bond_dev_list[i].bd_addr[2] << 8) + bond_dev_list[i].bd_addr[3],
				 (bond_dev_list[i].bd_addr[4] << 8) + bond_dev_list[i].bd_addr[5]);
	}
	free(bond_dev_list);
	return;
}

void app_main(void)
{
	if (eqmb_ble_semaphore == NULL)
		eqmb_ble_semaphore = xSemaphoreCreateMutex();
	// bt device init
	eqmb_nvs_flash_init();
	eqmb_bt_controller_init();
	eqmb_bluedroid_init();
	ESP_ERROR_CHECK(esp_ble_gap_set_rand_addr((uint8_t *)eqmb_static_local_addr));
	ESP_ERROR_CHECK(esp_hidd_profile_init());
	eqmb_load_bond_devices_as_whitelist();
	// register the callback function to the gap module
	esp_ble_gap_register_callback(gap_event_handler);
	esp_hidd_register_callbacks(hidd_event_callback);
	eqmb_gap_init();
	// init gpio
	eqmb_adv_gpio_init();

	// xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}
