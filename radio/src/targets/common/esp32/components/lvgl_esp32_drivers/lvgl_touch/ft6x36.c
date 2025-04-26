/*
* Copyright © 2020 Wolfgang Christl

* Permission is hereby granted, free of charge, to any person obtaining a copy of this
* software and associated documentation files (the “Software”), to deal in the Software
* without restriction, including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
* to whom the Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#include <esp_log.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include <lvgl.h>
#else
#include <lvgl/lvgl.h>
#endif
#include "ft6x36.h"
#include "lvgl_i2c/i2c_manager.h"

#define TAG "FT6X36"
#define FT6X36_TOUCH_QUEUE_ELEMENTS 1

#define FT6X36_I2C_TIMEOUT -1
static i2c_master_dev_handle_t ft6x06_handle = NULL;

static ft6x36_status_t ft6x36_status;
static ft6x36_touch_t touch_inputs = { -1, -1, LV_INDEV_STATE_REL };    // -1 coordinates to designate it was never touched
#if CONFIG_LV_FT6X36_COORDINATES_QUEUE
QueueHandle_t ft6x36_touch_queue_handle;
#endif

static esp_err_t ft6x06_i2c_read8(uint8_t register_addr, uint8_t *data_buf) {
    return i2c_master_transmit_receive(ft6x06_handle, &register_addr, 1, data_buf, 1, FT6X36_I2C_TIMEOUT);
}

static esp_err_t ft6x06_i2c_write8(uint8_t register_addr, uint8_t data) {
    uint8_t write_buf[2] = {register_addr, data};
    return i2c_master_transmit(ft6x06_handle, write_buf, 2, FT6X36_I2C_TIMEOUT);
}

/**
  * @brief  Read the FT6x36 gesture ID. Initialize first!
  * @param  dev_addr: I2C FT6x36 Slave address.
  * @retval The gesture ID or 0x00 in case of failure
  */
uint8_t ft6x36_get_gesture_id() {
    if (!ft6x36_status.inited) {
        ESP_LOGE(TAG, "Init first!");
        return 0x00;
    }
    uint8_t data_buf;
    esp_err_t ret;
    if ((ret = ft6x06_i2c_read8(FT6X36_GEST_ID_REG, &data_buf) != ESP_OK))
        ESP_LOGE(TAG, "Error reading from device: %s", esp_err_to_name(ret));
    return data_buf;
}

/**
  * @brief  Initialize for FT6x36 communication via I2C
  * @param  dev_addr: Device address on communication Bus (I2C slave address of FT6X36).
  * @retval None
  */
void ft6x06_init(uint16_t dev_addr) {

    ft6x36_status.inited = true;

    esp_err_t ret;
    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = 400000, // TODO: configurable?
        .device_address = dev_addr,
    };
    ret = i2c_master_bus_add_device(lvgl_i2c_bus_handle, &i2c_dev_conf, &ft6x06_handle);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "ft6x06_i2c_read8 err: %s", esp_err_to_name(ret));  

    uint8_t data_buf;
    ESP_LOGI(TAG, "Found touch panel controller");
    if ((ret = ft6x06_i2c_read8(FT6X36_PANEL_ID_REG, &data_buf)) != ESP_OK)
        ESP_LOGE(TAG, "ft6x06_i2c_read8 err: %s", esp_err_to_name(ret));    // Only show error the first time
    //ESP_LOGI(TAG, "\tDevice ID: 0x%02x", data_buf);

    if ((ret = ft6x06_i2c_read8(FT6X36_CHIPSELECT_REG, &data_buf)) != ESP_OK)
        ESP_LOGE(TAG, "ft6x06_i2c_read8 err: %s", esp_err_to_name(ret)); 
    //ESP_LOGI(TAG, "\tChip ID: 0x%02x", data_buf);

    if ((ret = ft6x06_i2c_read8(FT6X36_DEV_MODE_REG, &data_buf)) != ESP_OK)
        ESP_LOGE(TAG, "ft6x06_i2c_read8 err: %s", esp_err_to_name(ret)); 
    //ESP_LOGI(TAG, "\tDevice mode: 0x%02x", data_buf);

    if ((ret = ft6x06_i2c_read8(FT6X36_FIRMWARE_ID_REG, &data_buf)) != ESP_OK)
        ESP_LOGE(TAG, "ft6x06_i2c_read8 err: %s", esp_err_to_name(ret)); 
    //ESP_LOGI(TAG, "\tFirmware ID: 0x%02x", data_buf);

    if ((ret = ft6x06_i2c_read8(FT6X36_RELEASECODE_REG, &data_buf)) != ESP_OK)
        ESP_LOGE(TAG, "ft6x06_i2c_read8 err: %s", esp_err_to_name(ret)); 
    //ESP_LOGI(TAG, "\tRelease code: 0x%02x", data_buf);
    
    // Interrupt polling mode
    if ((ret = ft6x06_i2c_write8(FT6X36_G_MODE_REG, 0)) != ESP_OK)
       ESP_LOGE(TAG, "ft6x06_i2c_write8 err: %s", esp_err_to_name(ret)); 

#if CONFIG_LV_FT6X36_COORDINATES_QUEUE
    ft6x36_touch_queue_handle = xQueueCreate( FT6X36_TOUCH_QUEUE_ELEMENTS, sizeof( ft6x36_touch_t ) );
    if( ft6x36_touch_queue_handle == NULL )
    {
        ESP_LOGE( TAG, "\tError creating touch input FreeRTOS queue" );
        return;
    }
    xQueueSend( ft6x36_touch_queue_handle, &touch_inputs, 0 );
#endif
}

/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
bool ft6x36_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    if (!ft6x36_status.inited) {
        ESP_LOGE(TAG, "Init first!");
        return 0x00;
    }
    uint8_t data_buf[5];        // 1 byte status, 2 bytes X, 2 bytes Y
    uint8_t register_addr = FT6X36_TD_STAT_REG;
    esp_err_t ret = i2c_master_transmit_receive(ft6x06_handle, &register_addr, 1, data_buf, 5, FT6X36_I2C_TIMEOUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error talking to touch IC: %s", esp_err_to_name(ret));
    }
    // Number of detected touch points
    uint8_t touch_pnt_cnt = data_buf[0];  

    // ignore no touch & multi touch
    if (ret != ESP_OK || touch_pnt_cnt != 1) {    
        if ( touch_inputs.current_state != LV_INDEV_STATE_REL) {
            touch_inputs.current_state = LV_INDEV_STATE_REL;
#if CONFIG_LV_FT6X36_COORDINATES_QUEUE
            xQueueOverwrite( ft6x36_touch_queue_handle, &touch_inputs );
#endif
        } 
        data->point.x = touch_inputs.last_x;
        data->point.y = touch_inputs.last_y;
        data->state = touch_inputs.current_state;
        return false;
    }

    touch_inputs.current_state = LV_INDEV_STATE_PR;
    touch_inputs.last_x = ((data_buf[1] & FT6X36_MSB_MASK) << 8) | (data_buf[2] & FT6X36_LSB_MASK);
    touch_inputs.last_y = ((data_buf[3] & FT6X36_MSB_MASK) << 8) | (data_buf[4] & FT6X36_LSB_MASK);

#if CONFIG_LV_FT6X36_SWAPXY
    int16_t swap_buf = touch_inputs.last_x;
    touch_inputs.last_x = touch_inputs.last_y;
    touch_inputs.last_y = swap_buf;
#endif
#if CONFIG_LV_FT6X36_INVERT_X
    touch_inputs.last_x =  LV_HOR_RES - touch_inputs.last_x;
#endif
#if CONFIG_LV_FT6X36_INVERT_Y
    touch_inputs.last_y = LV_VER_RES - touch_inputs.last_y;
#endif
    data->point.x = touch_inputs.last_x;
    data->point.y = touch_inputs.last_y;
    data->state = touch_inputs.current_state;
    //ESP_LOGD(TAG, "X=%u Y=%u", data->point.x, data->point.y);

#if CONFIG_LV_FT6X36_COORDINATES_QUEUE
    xQueueOverwrite( ft6x36_touch_queue_handle, &touch_inputs );
#endif

    return false;
}
