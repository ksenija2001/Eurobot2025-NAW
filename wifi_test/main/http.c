#include "http.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

#define URL SERVER_URL
#define PORT SERVER_PORT

// static char *REQUEST = "GET / HTTP/1.0\r\n"
//         "Host: "+ SERVER_URL + ":" + SERVER_URL +"\r\n"
//         "User-Agent: esp-idf/1.0 esp32\r\n"
//         "\r\n";

char HTTP_REQUEST[256];

void just_test_it(){
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };

    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    while(1) {
        ESP_LOGW("Test", "Http");
        char buffer[25];
        itoa(SERVER_PORT, buffer, 10);
        ESP_LOGW("Test", "%s", buffer);
        int err = getaddrinfo(SERVER_URL, buffer, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE("HTTP", "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI("HTTP", "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE("HTTP", "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI("HTTP", "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE("HTTP", "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI("HTTP", "... connected");
        freeaddrinfo(res);

        snprintf(HTTP_REQUEST, sizeof(HTTP_REQUEST),
             "GET / HTTP/1.0\r\n"
             "Host: %s:%d\r\n"
             "User-Agent: esp-idf/1.0 esp32\r\n"
             "\r\n",
             SERVER_URL, SERVER_PORT);

        if (write(s, HTTP_REQUEST, strlen(HTTP_REQUEST)) < 0) {
            ESP_LOGE("HTTP", "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI("HTTP", "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE("HTTP", "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI("HTTP", "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI("HTTP", "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI("HTTP", "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI("HTTP", "Starting again!");
    }
}