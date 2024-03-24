/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_random.h"
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
#include "driver/rtc_io.h"
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

// use this lock to access below variables if necessary
static SemaphoreHandle_t eqmb_ble_semaphore = NULL;
// pairing status
static bool eqmb_pairing_on = false;
static bool eqmb_pairing_gpio_pressed = false;
static QueueHandle_t eqmb_pairing_gpio_queue = NULL;
static TimerHandle_t eqmb_pairing_timer = NULL;
// bluetooth status
static bool eqmb_connected = false; // bool = only one connection
									// otherwise, should use a counter
static uint16_t hidd_conn_id;
static esp_bd_addr_t eqmb_current_remote_addr = {0};
const static esp_bd_addr_t eqmb_static_local_addr = {0xd6, 0x3c, 0x1e, 0x0b, 0x73, 0x15};

// the RAGE-BUTTON!!!!! FINALLY
static QueueHandle_t eqmb_ragebtn_gpio_queue = NULL;
// this spec str list must have length of 2^n
#define ENABLE_SPEC_STR 1 // comment out this line to disable special string
const static uint8_t eqmb_ragebtn_spec_str_len = 4;
extern const keyboard_cmd_t **eqmb_ragebtn_spec_str[4];

const static gpio_num_t eqmb_ragebtn_gpio_num = GPIO_NUM_25;	 // ? button for april to rage-press
const static gpio_num_t eqmb_pairing_led_gpio_num = GPIO_NUM_26; // led to indicate advertising
const static gpio_num_t eqmb_pairing_gpio_num = GPIO_NUM_27;	 // button to start pairing

// sleep
const static gpio_num_t eqmb_sleep_gpio_num = eqmb_pairing_gpio_num; // use the pairing button for wakeup
static TimerHandle_t eqmb_sleep_timer = NULL;
const static int eqmb_sleep_timer_duration = 120000 * portTICK_PERIOD_MS; // 2 minutes = 120,000 ms

#define HIDD_DEVICE_NAME "EQMB"
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

static void IRAM_ATTR delay_microsecond(int64_t us)
{
	if (us == 0)
		return;
	int64_t start = esp_timer_get_time();
	int64_t end = start + us;
	if (end < start)
		// this is overflow, though we should not reach here since 2^63 us is
		// 292471 years
		return;
	while (esp_timer_get_time() < end)
		asm("nop"); // busy waiting, but ok within several us i guess
	return;
}

static void eqmb_sleep_timer_callback(TimerHandle_t xTimer)
{
	ESP_LOGI(TAG, "configuring sleep");
	// disable wifi and bluetooth
	esp_bluedroid_disable();
	esp_bt_controller_disable();
	// configure for wakeup
	esp_sleep_enable_ext0_wakeup(eqmb_sleep_gpio_num, 0);
	rtc_gpio_pullup_en(eqmb_sleep_gpio_num);
	rtc_gpio_pulldown_dis(eqmb_sleep_gpio_num);
	// start deepsleep
	ESP_LOGI(TAG, "start sleep");
	esp_deep_sleep_start();
	return;
}

static void eqmb_sleep_timer_init(void)
{
	eqmb_sleep_timer = xTimerCreate("eqmb_sleep_timer",
									eqmb_sleep_timer_duration,
									pdFALSE, NULL,
									eqmb_sleep_timer_callback);
	assert(eqmb_sleep_timer != NULL);
	xTimerStart(eqmb_sleep_timer, 0);
	return;
}

static void eqmb_pairing_led_task(void *arg)
{
	bool blink_status = false;
	bool led_on;
	while (1)
	{
		blink_status = !blink_status;
		led_on = eqmb_pairing_gpio_pressed || (blink_status && eqmb_pairing_on);
		gpio_set_level(GPIO_NUM_2, led_on);
		gpio_set_level(eqmb_pairing_led_gpio_num, led_on);
		vTaskDelay(200 / portTICK_PERIOD_MS);
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
	}
	xSemaphoreGive(eqmb_ble_semaphore);
	return;
}

static void eqmb_pairing_timer_callback(TimerHandle_t xTimer)
{
	eqmb_pairing_stop();
	return;
}

static void esp_ble_clear_bond_device(void)
{
	int bond_dev_num = esp_ble_get_bond_device_num();
	esp_ble_bond_dev_t *bond_dev_list = (esp_ble_bond_dev_t *)malloc(bond_dev_num * sizeof(esp_ble_bond_dev_t));
	esp_ble_get_bond_device_list(&bond_dev_num, bond_dev_list);
	for (int i = 0; i < bond_dev_num; i++)
		esp_ble_remove_bond_device(bond_dev_list[i].bd_addr);
	free(bond_dev_list);
	return;
}

static void eqmb_pairing_short_press_handler(void)
{
	if (!eqmb_pairing_on)
	{
		ESP_LOGI(TAG, "start pairing mode");
		eqmb_pairing_start();
	}
	else
	{
		ESP_LOGI(TAG, "stop pairing mode");
		eqmb_pairing_stop();
	}
	return;
}

static void eqmb_pairing_long_press_handler(void)
{
	// long press reset the bond devices connections, whitelist
	xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
	eqmb_pairing_on = false;
	xSemaphoreGive(eqmb_ble_semaphore);
	esp_ble_gap_disconnect(eqmb_current_remote_addr);
	esp_ble_clear_bond_device();
	esp_ble_gap_clear_whitelist();
	// add some delay for the bond device and whitelist update to take place
	vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms
	// then reset mcu
	// eqmb_pairing_start();
	esp_restart();
	return;
}

static void eqmb_pairing_gpio_task(void *arg)
{
	int64_t last_isr_time = 0, curr_isr_time;
	uint32_t press_time = 0;
	bool is_release_event;

	while (1)
	{
		if (xQueueReceive(eqmb_pairing_gpio_queue, NULL, portMAX_DELAY))
		{
			curr_isr_time = esp_timer_get_time();
			// jitter protection
			// if registered too quickly, ignore
			if (curr_isr_time - last_isr_time < 20000) // 20ms
				continue;
			// now consider it as a valid press
			last_isr_time = curr_isr_time;
			xTimerReset(eqmb_sleep_timer, 0);
			// delay 10ms then test the pin level
			delay_microsecond(10000); // 10ms debounce
			is_release_event = gpio_get_level(eqmb_pairing_gpio_num);
			if ((!eqmb_pairing_gpio_pressed) && (!is_release_event))
			{
				// start of press timing
				press_time = curr_isr_time;
				eqmb_pairing_gpio_pressed = true;
			}
			else if (eqmb_pairing_gpio_pressed && is_release_event)
			{
				// end of press timing
				eqmb_pairing_gpio_pressed = false;
				// check press duration (5s mark)
				press_time = curr_isr_time - press_time;
				if (press_time < 5000000)
				{
					ESP_LOGI(TAG, "short press detected");
					eqmb_pairing_short_press_handler();
				}
				else
				{
					ESP_LOGI(TAG, "long press detected");
					eqmb_pairing_long_press_handler();
				}
			}
			else
			{
				// something unexpected happened, but we need to reset the state
				eqmb_pairing_gpio_pressed = false;
			}
		}
	}
	return;
}

static void IRAM_ATTR eqmb_pairing_gpio_isr(void *arg)
{
	// jitter protection in handler
	xQueueSendFromISR(eqmb_pairing_gpio_queue, NULL, NULL);
	return;
}

static void eqmb_pairing_gpio_init(void)
{
	// leds
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(eqmb_pairing_led_gpio_num, GPIO_MODE_OUTPUT);
	// create tasks
	eqmb_pairing_gpio_queue = xQueueCreate(1, 0);
	assert(eqmb_pairing_gpio_queue != NULL);
	eqmb_pairing_timer = xTimerCreate("eqmb_pairing_timer",
									  30000 / portTICK_PERIOD_MS,
									  pdFALSE, NULL,
									  eqmb_pairing_timer_callback);
	assert(eqmb_pairing_timer != NULL);
	xTaskCreate(&eqmb_pairing_gpio_task, "eqmb_pairing_gpio_task", 2048, NULL, 5, NULL);
	xTaskCreate(&eqmb_pairing_led_task, "eqmb_pairing_led_task", 1024, NULL, 5, NULL);
	// pairing pin
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = 1ULL << eqmb_pairing_gpio_num;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
	gpio_isr_handler_add(eqmb_pairing_gpio_num, eqmb_pairing_gpio_isr, NULL);
	return;
}

static void eqmb_ragebtn_send_key(uint8_t act_streak)
{
	const static keyboard_cmd_t **sstr_word_p = NULL;	// NULL = send normal ?
	static int spec_str_due = 11;						// this is a magical number
	static keyboard_cmd_t qm_buf = {HID_KEY_FWD_SLASH}; // used send regular ?

#ifndef ENABLE_SPEC_STR
	for (int i = 0; i < act_streak; i++)
	{
		esp_hidd_send_keyboard_value(hidd_conn_id, 0x02, &qm_buf, 1);
		esp_hidd_send_keyboard_value(hidd_conn_id, 0x00, &qm_buf, 0);
	}
#else
	// re-fill value of spec_str_due
	if ((sstr_word_p == NULL) && (spec_str_due == 0))
	{
		spec_str_due = esp_random() & 0x1f;
		if (spec_str_due < 0x08) // at least 8 presses to trigger special string
			spec_str_due += 0x08;
	}
	// check if need to send special string
	if (sstr_word_p == NULL)
	{
		// send regular ?, act_streak increase the number of ?
		for (int i = 0; i <= act_streak; i++)
		{
			esp_hidd_send_keyboard_value(hidd_conn_id, 0x02, &qm_buf, 1);
			esp_hidd_send_keyboard_value(hidd_conn_id, 0x00, &qm_buf, 0);
		}
		spec_str_due--;
		if ((spec_str_due == 0) && (eqmb_ragebtn_spec_str_len > 0))
		{
			// next press will send special string
			// determine the next special string
			sstr_word_p = eqmb_ragebtn_spec_str[esp_random() & (eqmb_ragebtn_spec_str_len - 1)];
		}
	}
	else
	{
		// send special string
		if (!*sstr_word_p)
			return;
		// send the keys
		const keyboard_cmd_t *c = *sstr_word_p;
		while (*c) // NULL-terminated
		{
			key_mask_t mask = ((*c == HID_KEY_SPACEBAR) || (*c == HID_KEY_SGL_QUOTE) || (*c == HID_KEY_DOT)) ? 0x00 : 0x02;
			esp_hidd_send_keyboard_value(hidd_conn_id, mask, (keyboard_cmd_t *)c, 1);
			esp_hidd_send_keyboard_value(hidd_conn_id, 0x00, (keyboard_cmd_t *)c, 0);
			c++;
		}
		// move to next word
		sstr_word_p++;
		if (!*sstr_word_p)
			// end of the special string
			sstr_word_p = NULL;
	}
#endif // ENABLE_SPEC_STR
	return;
}

static void eqmb_ragebtn_gpio_task(void *arg)
{
	int64_t last_rage_time = 0, curr_isr_time; // last_rage_time = last_isr_time
	uint8_t act_streak = 0;
	while (1)
	{
		if (xQueueReceive(eqmb_ragebtn_gpio_queue, NULL, portMAX_DELAY))
		{
			curr_isr_time = esp_timer_get_time();
			// jitter protection
			// if registered too quickly, ignore
			if (curr_isr_time - last_rage_time < 50000) // 50ms
				continue;
			// delay 10ms then test the pin level
			delay_microsecond(10000); // 10ms debounce
			if (gpio_get_level(eqmb_ragebtn_gpio_num) == 1)
				continue;
			// now consider it as a valid press
			xTimerReset(eqmb_sleep_timer, 0);
			// we still need the last_rage_time for the streak detection, so
			// no update here
			if (!eqmb_connected)
				continue;
			if (curr_isr_time - last_rage_time < 750000)
			{
				// 750ms grace period for 'rage mode', considering the agility
				// stat of the certain professor

				// streak is exiciting! need 16 !s
				ESP_LOGI(TAG, "RAGE BUTTON STREAK!!!!!!!!!!!!!!!!");
				act_streak += (act_streak < 5) ? 1 : 0; // max 5, for 6 chars
			}
			else
			{
				// a little bit of boring here, tbh; so only 8 !s
				ESP_LOGI(TAG, "RAGE BUTTON PRESSED!!!!!!!!");
				act_streak = 0;
			}
			// update last_rage_time here
			last_rage_time = curr_isr_time;
			// 0x02 = LSHIFT modifier
			// use time as the random number
			eqmb_ragebtn_send_key(act_streak);
		}
	}
	return;
}

static void IRAM_ATTR eqmb_ragebtn_gpio_isr(void *arg)
{
	// jitter protection in handler
	xQueueSendToBackFromISR(eqmb_ragebtn_gpio_queue, NULL, pdFALSE);
	return;
}

static void eqmb_ragebtn_gpio_init(void)
{
	// create tasks
	eqmb_ragebtn_gpio_queue = xQueueCreate(1, 0);
	assert(eqmb_ragebtn_gpio_queue != NULL);
	xTaskCreate(&eqmb_ragebtn_gpio_task, "eqmb_ragebtn_gpio_task", 2048, NULL, 10, NULL);
	// rage button pin
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_NEGEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = 1ULL << eqmb_ragebtn_gpio_num;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
	// gpio_install_isr_service(0);
	gpio_isr_handler_add(eqmb_ragebtn_gpio_num, eqmb_ragebtn_gpio_isr, NULL);
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
		hidd_conn_id = param->connect.conn_id;
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
		if (!param->ble_security.auth_cmpl.success)
		{
			ESP_LOGE(TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
			break;
		}
		// disconnect the old device when connected if applicable
		if (eqmb_connected)
		{
			esp_ble_gap_disconnect(eqmb_current_remote_addr);
			esp_ble_remove_bond_device(eqmb_current_remote_addr);
			esp_ble_gap_update_whitelist(false, eqmb_current_remote_addr, BLE_ADDR_TYPE_PUBLIC);
		}
		// update new connection status
		xSemaphoreTake(eqmb_ble_semaphore, portMAX_DELAY);
		eqmb_connected = true;
		memcpy(eqmb_current_remote_addr, bd_addr, sizeof(esp_bd_addr_t));
		xSemaphoreGive(eqmb_ble_semaphore);
		esp_ble_gap_update_whitelist(true, bd_addr, BLE_ADDR_TYPE_PUBLIC);
		eqmb_pairing_stop();
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
		ESP_LOGI(TAG, "adding bond device to whitelist: %08x%04x",
				 (bond_dev_list[i].bd_addr[0] << 24) + (bond_dev_list[i].bd_addr[1] << 16) + (bond_dev_list[i].bd_addr[2] << 8) + bond_dev_list[i].bd_addr[3],
				 (bond_dev_list[i].bd_addr[4] << 8) + bond_dev_list[i].bd_addr[5]);
	}
	free(bond_dev_list);
	return;
}

void app_main(void)
{
	if (esp_sleep_get_wakeup_cause())
		// just reset, don't bother with other stuff
		esp_restart();

	eqmb_ble_semaphore = xSemaphoreCreateMutex();
	assert(eqmb_ble_semaphore != NULL);
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
	eqmb_sleep_timer_init();
	gpio_install_isr_service(0);
	eqmb_pairing_gpio_init();
	eqmb_ragebtn_gpio_init();
	return;
}