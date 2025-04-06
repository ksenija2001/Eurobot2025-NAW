#include "main.h"

Socket rpi_socket = {
    .tag = "RPi"
};

I2C_Bus bus;
TOF tof = {
    .tag = "TOF"
};

void app_main(void)
{
    init_i2c_bus(&bus, GPIO_NUM_4, GPIO_NUM_5, 7);
    init_tof(&tof, &bus, I2C_TOF_ADDRESS, GPIO_NUM_6);

    // init_wifi(WIFI_SSID, WIFI_PASS);
    // init_socket(&rpi_socket);

    // socket_connect(&rpi_socket, SERVER_IP, SERVER_PORT);

    // socket_recv(&rpi_socket);
    // socket_close(&rpi_socket);

    // if(!strcmp("Sima 1", rpi_socket.rx_buff)){
    //     ESP_LOGE(rpi_socket.tag, "Running aplication 1: %s", rpi_socket.rx_buff);
    // }else{
    //     ESP_LOGE(rpi_socket.tag, "Error receiving sima code: %s", rpi_socket.rx_buff);
    // }
    
    while(1){  
        // ESP_LOGI("App", "Running...");
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}
