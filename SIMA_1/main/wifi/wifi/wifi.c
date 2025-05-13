#include "wifi.h"

static wifi_init_config_t   wifi_init;
static wifi_config_t        wifi_conf;

void init_wifi(char* ssid, char* pass){

    #if defined(DEBUG_WIFI) && DEBUG_WIFI_LEVEL == HIGH_DEBUG_WIFI_LEVEL
        ESP_LOGI(WIFI_TAG, "Initializing WiFi...");
    #endif

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    wifi_init = init;
    
    esp_wifi_init(&wifi_init);

    wifi_config_t conf = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .sae_h2e_identifier = "",
        }
    };

    wifi_conf = conf;
    wifi_set_ssid(ssid);
    wifi_set_pass(pass);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_conf);

    wifi_connect();
}

void wifi_set_ssid(char* ssid){
    #if defined(DEBUG_WIFI) && DEBUG_WIFI_LEVEL == LOW_DEBUG_WIFI_LEVEL
        ESP_LOGI(WIFI_TAG, "Setting WiFi SSID to %s", ssid);
    #endif
    memcpy(&wifi_conf.sta.ssid, ssid, strlen(ssid));
}

void wifi_set_pass(char* pass){
    #if defined(DEBUG_WIFI) && DEBUG_WIFI_LEVEL == LOW_DEBUG_WIFI_LEVEL
        ESP_LOGI(WIFI_TAG, "Setting WiFi password to %s", pass);
    #endif
    memcpy(&wifi_conf.sta.password, pass, strlen(pass));
}

void wifi_connect(){

    #if defined(DEBUG_WIFI) && DEBUG_WIFI_LEVEL == HIGH_DEBUG_WIFI_LEVEL
        ESP_LOGI(WIFI_TAG, "Connecting to WiFi...");
    #endif

    esp_wifi_start();

    while(true){
        esp_wifi_connect();

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            #if defined(DEBUG_WIFI) && DEBUG_WIFI_LEVEL == HIGH_DEBUG_WIFI_LEVEL
                ESP_LOGI(WIFI_TAG, "Connected to WiFi, RSSI: %d", ap_info.rssi);
            #endif
            break;
        }
        #if defined(DEBUG_WIFI) && DEBUG_WIFI_LEVEL == HIGH_DEBUG_WIFI_LEVEL
            ESP_LOGI(WIFI_TAG, "Waiting for WiFi connection...");
        #endif
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
    #if defined(DEBUG_WIFI) && DEBUG_WIFI_LEVEL == HIGH_DEBUG_WIFI_LEVEL
        ESP_LOGI(WIFI_TAG, "WiFi Connected");
    #endif
}