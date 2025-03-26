#include "main.h"

Socket rpi_socket = {
    .tag = "RPi"
};

void app_main(void)
{
    init_wifi(WIFI_SSID, WIFI_PASS);
    init_socket(&rpi_socket);

    socket_connect(&rpi_socket, SERVER_IP, SERVER_PORT);

    socket_recv(&rpi_socket);
    socket_close(&rpi_socket);

    while(1){  
        ESP_LOGI("App", "Running...");

        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}
