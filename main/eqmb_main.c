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
static bool eqmb_adv_ready = false;
static bool eqmb_adv_on = false;
static uint16_t hid_conn_id = 0;
static esp_bd_addr_t hid_conn_remote_addr = {0};
static bool sec_conn = false;

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static TaskHandle_t eqmb_adv_led_blink_task_handle = NULL;

#define EQMB_RAGEBTN_GPIO (GPIO_NUM_18)
#define EQMB_ADV_LED_GPIO (GPIO_NUM_19)
#define EQMB_CONNECT_GPIO (GPIO_NUM_21)

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME "HID"
static uint8_t hidd_service_uuid128[] = {
	/* LSB <--------------------------------------------------------------------------------> MSB */
	// first uuid, 16bit, [12],[13] is the value
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
	.appearance = 0x03c0,	// HID Generic,
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
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
	//.peer_addr            =
	//.peer_addr_type       =
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void eqmb_adv_led_blink_task(void *arg)
{
	bool led_on = false;
	while (1)
	{
		led_on = !led_on;
		gpio_set_level(GPIO_NUM_2, led_on);		   // internal led
		gpio_set_level(EQMB_ADV_LED_GPIO, led_on); // custom led
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
	return;
}

static void eqmb_connect_gpio_handler(void *arg)
{
	xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
	ESP_LOGI(TAG, "Connect button released");
	xSemaphoreGive(eqmb_ble_semaphore);
	// if (eqmb_adv_on)
	//{
	//	eqmb_adv_on = false;
	//	esp_ble_gap_stop_advertising();
	//	vTaskDelete(eqmb_adv_led_blink_task_handle);
	// }
	// else
	//{
	//	if (sec_conn)
	//		esp_ble_gap_disconnect(hid_conn_remote_addr);
	//	eqmb_adv_on = true;
	//	esp_ble_gap_start_advertising(&hidd_adv_params);
	//	xTaskCreate(eqmb_adv_led_blink_task, "eqmb_adv_led_blink_task", 1024,
	//				NULL, 5, &eqmb_adv_led_blink_task_handle);
	// }
	// xSemaphoreGive(eqmb_ble_semaphore);
	return;
}

static void eqmb_connect_gpio_init(void)
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = 1ULL << EQMB_CONNECT_GPIO;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 1;
	ESP_ERROR_CHECK(gpio_config(&io_conf));
	ESP_ERROR_CHECK(gpio_install_isr_service(0));
	ESP_ERROR_CHECK(gpio_isr_handler_add(EQMB_CONNECT_GPIO, eqmb_connect_gpio_handler, NULL));
	return;
}

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
	switch (event)
	{
	case ESP_HIDD_EVENT_REG_FINISH:
	{
		if (param->init_finish.state == ESP_HIDD_INIT_OK)
		{
			// esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
			esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
			esp_ble_gap_config_adv_data(&hidd_adv_data);
			xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
			eqmb_adv_ready = true;
			xSemaphoreGive(eqmb_ble_semaphore);
		}
		break;
	}
	case ESP_BAT_EVENT_REG:
		break;
	case ESP_HIDD_EVENT_DEINIT_FINISH:
		break;
	case ESP_HIDD_EVENT_BLE_CONNECT:
	{
		ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
		xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
		hid_conn_id = param->connect.conn_id;
		memcpy(hid_conn_remote_addr, param->connect.remote_bda, sizeof(esp_bd_addr_t));
		xSemaphoreGive(eqmb_ble_semaphore);
		break;
	}
	case ESP_HIDD_EVENT_BLE_DISCONNECT:
	{
		xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
		sec_conn = false;
		xSemaphoreGive(eqmb_ble_semaphore);
		ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
		// esp_ble_gap_start_advertising(&hidd_adv_params);
		break;
	}
	case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT:
	{
		ESP_LOGI(TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
		ESP_LOG_BUFFER_HEX(TAG, param->vendor_write.data, param->vendor_write.length);
		break;
	}
	case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT:
	{
		ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
		ESP_LOG_BUFFER_HEX(TAG, param->led_write.data, param->led_write.length);
		break;
	}
	default:
		break;
	}
	return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event)
	{
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		// esp_ble_gap_start_advertising(&hidd_adv_params);
		break;
	case ESP_GAP_BLE_SEC_REQ_EVT:
		for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
			ESP_LOGD(TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		break;
	case ESP_GAP_BLE_AUTH_CMPL_EVT:
		xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
		sec_conn = true;
		xSemaphoreGive(eqmb_ble_semaphore);
		esp_bd_addr_t bd_addr;
		memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
		ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x",
				 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
				 (bd_addr[4] << 8) + bd_addr[5]);
		ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
		ESP_LOGI(TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
		if (!param->ble_security.auth_cmpl.success)
			ESP_LOGE(TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
		break;
	default:
		break;
	}
}

void hid_demo_task(void *pvParameters)
{
	return;
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

void app_main(void)
{
	if (eqmb_ble_semaphore == NULL)
		eqmb_ble_semaphore = xSemaphoreCreateMutex();
	// bt device init
	eqmb_nvs_flash_init();
	eqmb_bt_controller_init();
	eqmb_bluedroid_init();
	ESP_ERROR_CHECK(esp_hidd_profile_init());
	// register the callback function to the gap module
	esp_ble_gap_register_callback(gap_event_handler);
	esp_hidd_register_callbacks(hidd_event_callback);
	eqmb_gap_init();
	// init gpio
	eqmb_connect_gpio_init();

	// xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}
