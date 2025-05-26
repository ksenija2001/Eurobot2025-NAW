#ifndef WIFI_H
#define WIFI_H

#include <string.h>

#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"

//#define DEBUG_WIFI
#define HIGH_DEBUG_WIFI_LEVEL 1
#define LOW_DEBUG_WIFI_LEVEL 0

#if defined(DEBUG_WIFI)
    #include "esp_log.h"

    #define WIFI_TAG "WiFi"
#endif

#if !defined(DEBUG_WIFI_LEVEL)
    #define DEBUG_WIFI_LEVEL HIGH_DEBUG_WIFI_LEVEL
#endif

/***
 * @brief Function used to initialize WiFi
 * 
 * @param ssid Name of WiFi network to connect to (string)
 * @param pass Password for WiFi network (string)
 * 
 * @retval None
 */
void init_wifi(char* ssid, char* pass);

/***
 * @brief FUNCTION NOT IN FUNCTION
 * 
 * Function used to set ssid of WiFi 
 * 
 * @param ssid Name of WiFi network (string)
 * 
 * @retval None
 */
void wifi_set_ssid(char* ssid);

/***
 * @brief FUNCTION NOT IN FUNCTION
 * 
 * Function used to set password for WiFi network
 * 
 * @param pass Password for WiFi network (string)
 * 
 * @retval None
 */
void wifi_set_pass(char* pass);

/***
 * @brief Function used to connect to WiFi after initialization automatically
 * 
 * @retval None
 */
void wifi_connect();

#endif //WIFI_H