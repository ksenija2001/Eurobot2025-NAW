#ifndef WIFI_H
#define WIFI_H

#include <string.h>

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "credentials.h"

void wifi_init();

#endif //WIFI_H