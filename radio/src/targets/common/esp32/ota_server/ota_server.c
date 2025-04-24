/*
* Copyright (C) OpenTX
*
* Based on code named
*   https://github.com/yanbe/esp32-ota-server
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

#include <string.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "ota_server.h"

static const char * TAG = "OTA";

/*socket*/
static int connect_socket = 0;
static int server_socket = 0;
static QueueHandle_t ota_mutex = NULL;
static volatile enum OtaState otaState = OTA_IDLE;

static int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    int err = getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen);
    if (err == -1) {
        ESP_LOGE(TAG, "getsockopt failed:%s", strerror(err));
        return -1;
    }
    return result;
}

static int show_socket_error_reason(const char *str, int socket)
{
    xSemaphoreTake(ota_mutex, portMAX_DELAY);
    if(OTA_STOPPING == otaState){
        xSemaphoreGive(ota_mutex);
        return 0;
    } else {
        xSemaphoreGive(ota_mutex);
    }
    int err = get_socket_error_code(socket);

    if (err != 0) {
        ESP_LOGW(TAG, "%s socket error %d %s", str, err, strerror(err));
    }

    return err;
}

static esp_err_t create_tcp_server()
{
    ESP_LOGI(TAG, "server socket....port=%d", OTA_LISTEN_PORT);
    struct sockaddr_in server_addr;
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        show_socket_error_reason("create_server", server_socket);
        return ESP_FAIL;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(OTA_LISTEN_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        show_socket_error_reason("bind_server", server_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    if (listen(server_socket, 5) < 0) {
        show_socket_error_reason("listen_server", server_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    struct sockaddr_in client_addr;
    unsigned int socklen = sizeof(client_addr);
    connect_socket = accept(server_socket, (struct sockaddr *)&client_addr, (socklen_t *)&socklen);
    if (connect_socket < 0) {
        show_socket_error_reason("accept_server", connect_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    /*connection established，now can send/recv*/
    ESP_LOGI(TAG, "tcp connection established!");
    return ESP_OK;
}

void ota_server_start()
{   
    if (ota_mutex == NULL) ota_mutex = xSemaphoreCreateMutex();
    xSemaphoreTake(ota_mutex, portMAX_DELAY);
    otaState = OTA_WAITING;
    xSemaphoreGive(ota_mutex);
    esp_err_t err = create_tcp_server();
    if(ESP_OK != err){
        return;
    }
    xSemaphoreTake(ota_mutex, portMAX_DELAY);
    otaState = OTA_UPDATING;
    xSemaphoreGive(ota_mutex);
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
        update_partition->subtype, update_partition->address);

    int recv_len;
    char ota_buff[OTA_BUFF_SIZE] = {0};
    bool is_req_body_started = false;
    int content_length = -1;
    int content_received = 0;

    esp_ota_handle_t ota_handle = 0;
    do {
        recv_len = recv(connect_socket, ota_buff, OTA_BUFF_SIZE, 0);
        if (recv_len > 0) {
            if (!is_req_body_started) {
                const char *content_length_start = "Content-Length: ";
                char *content_length_start_p = strstr(ota_buff, content_length_start) + strlen(content_length_start);
                sscanf(content_length_start_p, "%d", &content_length);
                ESP_LOGI(TAG, "Detected content length: %d", content_length);
                err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
                if( ESP_OK != err){
                    ESP_LOGE(TAG, "Failed to update OTA partition: %s", esp_err_to_name(err));
                    break;
                }
                const char *header_end = "\r\n\r\n";
                char *body_start_p = strstr(ota_buff, header_end) + strlen(header_end);
                int body_part_len = recv_len - (body_start_p - ota_buff);
                esp_ota_write(ota_handle, body_start_p, body_part_len);
                content_received += body_part_len;
                is_req_body_started = true;
            } else {
                esp_ota_write(ota_handle, ota_buff, recv_len);
                content_received += recv_len;
            }
        }
        else if (recv_len < 0) {
            ESP_LOGE(TAG, "Error: recv data error! errno=%d", errno);
        }
    } while (recv_len > 0 && content_received < content_length);

    ESP_LOGI(TAG, "Binary transferred finished: %d bytes", content_received);
    if( 0 != ota_handle){
        err = esp_ota_end(ota_handle);
        if(ESP_OK == err){
            err = esp_ota_set_boot_partition(update_partition);
        }
    } 
    char res_buff[60];
    int send_len;
    if (err == ESP_OK) {
        send_len = sprintf(res_buff, "200 OK\n\nSuccess. Next boot partition is %s\n", update_partition->label);
    } else {
        send_len = sprintf(res_buff, "400 Bad Request\n\nFailure. Error code: 0x%x\n", err);
        ESP_LOGE(TAG, "Failed to update OTA partition: %s", esp_err_to_name(err));
    }
    send(connect_socket, res_buff, send_len, 0);
    close(connect_socket);
    close(server_socket);
    if (err == ESP_OK){
        const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
        ESP_LOGI(TAG, "Next boot partition subtype %d at offset 0x%x",
            boot_partition->subtype, boot_partition->address);
        ESP_LOGI(TAG, "Prepare to restart system!");
        vTaskDelay(2000/portTICK_PERIOD_MS);
        esp_restart();
    }
}

void ota_server_stop(){
    if(ota_mutex){
        enum OtaState currOtaState;
        do{
            xSemaphoreTake(ota_mutex, portMAX_DELAY);
            if(OTA_UPDATING != otaState){
                ESP_LOGI(TAG, "Stopping OTA server ...");
                otaState = OTA_STOPPING;
                close(server_socket);
            }
            currOtaState=otaState;
            xSemaphoreGive(ota_mutex);
            vTaskDelay(100/portTICK_PERIOD_MS);
        } while(OTA_UPDATING == currOtaState);
    }
}