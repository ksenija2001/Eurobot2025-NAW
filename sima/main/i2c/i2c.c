#include "i2c.h"

#include "esp_log.h"

uint8_t buffer[40000];

void init_i2c_bus(I2C_Bus* bus, uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t glitch_ignore_cnt){
    if(glitch_ignore_cnt != 7 && glitch_ignore_cnt != 10) return;
    
    bus->bus_config.i2c_port = 0;
    bus->bus_config.sda_io_num = SDA_PIN;
    bus->bus_config.scl_io_num = SCL_PIN;
    bus->bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus->bus_config.glitch_ignore_cnt = glitch_ignore_cnt;
    bus->bus_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus->bus_config, &bus->bus_handle));
}

void init_i2c_device(I2C_Bus* bus, I2C_Device* dev, uint8_t dev_address, uint32_t speed){
    dev->dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev->dev_config.device_address = dev_address;
    dev->dev_config.scl_speed_hz = speed;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus->bus_handle, &dev->dev_config, &dev->dev_handle));
}

esp_err_t i2c_device_alive(I2C_Bus* bus, uint16_t address){
    return i2c_master_probe(bus->bus_handle, address, 1000);
}

void i2c_send(I2C_Device* dev, uint16_t address, uint8_t* data, uint16_t len){
    i2c_master_transmit(dev->dev_handle, data, len, 100);
}

void i2c_sendByte(I2C_Device* dev, uint8_t* data){
    i2c_master_transmit(dev->dev_handle, data, 1, 100);
}

void i2c_sendWord(I2C_Device* dev, uint8_t* data){
    i2c_master_transmit(dev->dev_handle, data, 2, 100);
}

void i2c_receive(I2C_Device* dev, uint16_t address, uint8_t* data, uint16_t len){

}

void i2c_receiveByte(I2C_Device* dev, uint8_t* buff){
    i2c_master_receive(dev->dev_handle, buff, 1, 100);
}

void i2c_receiveWord(I2C_Device* dev, uint8_t* buff){
    i2c_master_receive(dev->dev_handle, buff, 2, 100);
}

int32_t i2c_send_RS16(I2C_Device* dev, uint16_t reg, uint8_t* data, uint32_t len){   
    buffer[1] = reg;
    buffer[0] = reg >> 8;
    memcpy(&(buffer[2]), data, len);

    #ifdef DEBUG_I2C
        ESP_LOGI("I2C", "Sending to device %x, register %x, data len %lu", dev->dev_config.device_address, reg, len);
    #endif

    i2c_master_transmit(dev->dev_handle, buffer, 2, 100);
    i2c_master_transmit(dev->dev_handle, &buffer[2], len, 100);

    return 0;
}


int32_t i2c_receive_RS16(I2C_Device* dev, uint16_t reg, uint8_t* buff, uint16_t len){
    uint8_t new_buff[2];
    new_buff[0] = reg >> 8;
    new_buff[1] = reg;
    i2c_master_transmit_receive(dev->dev_handle, new_buff, 2, buff, len, 100);
    return 0;
}