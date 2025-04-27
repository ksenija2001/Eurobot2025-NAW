#ifndef WIFI_H
#define WIFI_H

#include <string.h>

#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"

#include "esp_log.h"
#include "esp_mac.h"

void init_wifi();

void wifi_set_ssid(char* ssid);
void wifi_set_pass(char* pass);

void wifi_connect();

#endif //WIFI_H