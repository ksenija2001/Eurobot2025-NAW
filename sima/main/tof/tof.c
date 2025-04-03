#include "tof.h"

void init_tof(TOF* tof, I2C_Bus* bus, uint8_t address, uint8_t rst){

#if defined(DEBUG_TOF)
    ESP_LOGI(tof->tag, "Initializing TOF with address 0x%x", address);
#endif

    tof->bus = bus;
    tof->speed = 400000;
    
    init_i2c_device(bus, &(tof->device), address, tof->speed);
    init_gpio(GPIO_MODE_OUTPUT, GPIO_INTR_DISABLE, GPIO_NUM_6, GPIO_PULLDOWN_DISABLE, GPIO_PULLUP_DISABLE);

    tof->i2c_rst_pin = rst;
    tof->address = address;

#if defined(DEBUG_TOF)
    ESP_LOGI(tof->tag, "Reseting I2C driver");
#endif

    gpio_reset(tof->i2c_rst_pin);
    vTaskDelay(pdMS_TO_TICKS(500));

    gpio_set(tof->i2c_rst_pin);
    vTaskDelay(pdMS_TO_TICKS(500));

    gpio_reset(tof->i2c_rst_pin);
    vTaskDelay(pdMS_TO_TICKS(500));

    tof_is_alive(tof);
}

void tof_is_alive(TOF* tof){
    esp_err_t err = i2c_device_alive(tof->bus, tof->address);
    
    if(err == ESP_OK){
        #if defined(DEBUG_TOF)
            ESP_LOGI(tof->tag, "alive");
        #endif
    }else{
        #if defined(DEBUG_TOF)
            ESP_LOGE(tof->tag, "not alive");
        #endif
    }
}

void tof_configure(TOF* tof){

}

void tof_send(TOF* tof){

}

void tof_receive(TOF* tof){

}