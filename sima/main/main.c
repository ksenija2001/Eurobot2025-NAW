#include "main.h"

void app_main(void)
{
    init_wifi(WIFI_SSID, WIFI_PASS);

    while(1){  
        ESP_LOGI("App", "Running...");
        init_socket();
        socket_connect(SERVER_IP, SERVER_PORT);

        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}
