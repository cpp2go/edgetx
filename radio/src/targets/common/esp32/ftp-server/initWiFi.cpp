/*
 * Copyright (C) OpenTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */ 
 
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_task.h"
#define HASASSERT
#include "opentx.h"
#include "WiFi.h"

extern "C" {
#include "ftp.h"
#include "ota_server.h"
}

#define FTP_SERVER_TASK_CORE    1
#define OTA_SERVER_TASK_CORE    1
#define FTP_SERVER_STACK_SIZE   0x1000
#define OTA_SERVER_STACK_SIZE   0xE00
#define STA_CONNECT_TMO 10000

EXT_RAM_BSS_ATTR StackType_t ftp_task_stack[FTP_SERVER_STACK_SIZE];
EXT_RAM_BSS_ATTR StackType_t ota_task_stack[OTA_SERVER_STACK_SIZE];

static const char *TAG = "initWiFi.cpp";
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;
StaticTask_t wifiTaskBuffer;
StaticTask_t otaTaskBuffer;
static char ssid[sizeof(g_eeGeneral.wifi_ssid)+1] = "EdgeTX";
static char passwd[sizeof(g_eeGeneral.wifi_password)+1] = "";
char ftp_pass[FTP_USER_PASS_LEN_MAX > sizeof(g_eeGeneral.ftppass) ? FTP_USER_PASS_LEN_MAX +1 : sizeof(g_eeGeneral.ftppass)+1] = "edgetx";
SemaphoreHandle_t wifi_mutex = NULL;
static bool wifiESPNow = false;
static bool wifiServers = false;


void initWiFi()
{
    wifi_mutex = xSemaphoreCreateMutex();
    init_wifi();
}

void wifiTask(void *pvParameters)
{
    wifi_init_sta(ssid,passwd);
    int x = 0;
    while ((wifiState!=WIFI_CONNECTED) && (x++ < STA_CONNECT_TMO)) {
        vTaskDelay(portTICK_PERIOD_MS);
    }
    if(wifiState!=WIFI_CONNECTED) {
        ESP_LOGW(TAG, "Timeout connecting to '%s'",g_eeGeneral.wifi_ssid);
        wifi_init_softap();
    }
    ftpServerTask(pvParameters);
    ota_server_stop();
    if(wifiESPNow){
        startWiFiESPNow();
        resume_espnow();
    } else {
        ESP_LOGI(TAG, "Stopping WiFi ...");
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_stop());
    }
    wifiServers = false;
    wifiTaskHandle = NULL;
    wifiState=WIFI_IDLE;
    vTaskDelete(NULL);
}

static void ota_server_task(void * param)
{
    while(!(wifiState & WIFI_CONNECTED)){
        if(wifiState & WIFI_IDLE ){
            goto exit;
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    ota_server_start();
exit:    
    otaTaskHandle = NULL;
    vTaskDelete(NULL);
}

void startWiFiESPNow(){
    wifi_init_sta((char *)"", (char *)"");
    ESP_ERROR_CHECK_WITHOUT_ABORT( esp_wifi_set_channel(g_model.moduleData[INTERNAL_MODULE].espnow.ch, (wifi_second_chan_t)0) );
    wifiESPNow = true;
}

void stopWiFiESPNow(){
    ESP_LOGI(TAG, "Stopping WiFi (ESP-NOW ) ...");
    if (!wifiServers) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_stop());
    }
    wifiESPNow = false;
}

void startWiFi( char *ssid_zchar, char *passwd_zchar, char* ftppass_zchar)
{
    if(!(wifiState & WIFI_IDLE) && wifiServers) return;
    pause_espnow();
    wifiServers = true;
    wifiState = WIFI_STARTING;
    if(NULL != ssid_zchar) {
        //zchar2str(ssid,ssid_zchar,sizeof(g_eeGeneral.wifi_ssid));
        //zchar2str(passwd,passwd_zchar,sizeof(g_eeGeneral.wifi_password));
        //zchar2str(ftp_pass,ftppass_zchar,sizeof(g_eeGeneral.ftppass));
        strncpy(ssid,ssid_zchar,sizeof(g_eeGeneral.wifi_ssid));
        strncpy(passwd,passwd_zchar,sizeof(g_eeGeneral.wifi_password));
        strncpy(ftp_pass,ftppass_zchar,sizeof(g_eeGeneral.ftppass));
    }
    wifiTaskHandle = xTaskCreateStaticPinnedToCore( wifiTask, "WiFiTask", FTP_SERVER_STACK_SIZE, NULL,
            ESP_TASK_PRIO_MAX - 4, ftp_task_stack, &wifiTaskBuffer, FTP_SERVER_TASK_CORE );
    if( wifiTaskHandle == NULL){
        ESP_LOGE(TAG, "Failed to create task: 'WiFiTask'");
    }
    configASSERT( wifiTaskHandle );
    otaTaskHandle = xTaskCreateStaticPinnedToCore( ota_server_task, "OTATask", OTA_SERVER_STACK_SIZE,
            NULL, ESP_TASK_PRIO_MAX - 9, ota_task_stack, &otaTaskBuffer, OTA_SERVER_TASK_CORE );
    if( otaTaskHandle == NULL ){
        ESP_LOGE(TAG, "Failed to create task: 'OTATask'");
    }
    configASSERT( otaTaskHandle );
}


void stopWiFi()
{
    if(wifiState & (WIFI_STOPPING | WIFI_IDLE)) return;
    if(stop_wifi()) {
        wifiState = WIFI_STOPPING;
    }
}

const char* getWiFiStatus()
{
    if(wifiESPNow && !wifiServers){
        return "ESP-NOW";
    }
    static char stat[STATUS_LEN]="Idle";
    switch(wifiState) {
    case WIFI_IDLE:
        return "Idle";
        break;
    case WIFI_STARTING:
        return "Starting...";
        break;
    case WIFI_CONNECTING:
        return "Connecting...";
        break;
    case WIFI_CONNECTED:
        esp_ip4_addr_t tmp;
        tmp.addr=network_hasip();
        esp_ip4addr_ntoa(&tmp, stat, sizeof(stat));
        return stat;
    case WIFI_STOPPING:
        return "Stopping...";
        break;
    }
    return stat;
}

bool isWiFiStarted(uint32_t expire)
{
    expireTimer_ms=mp_hal_ticks_ms()+expire;
    return !(wifiState & WIFI_IDLE) && wifiServers;
}