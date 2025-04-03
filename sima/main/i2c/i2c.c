#include "i2c.h"

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

void i2c_send(I2C_Device* dev, uint16_t address, uint8_t* data, uint8_t len){

}

void i2c_sendByte(I2C_Device* dev, uint16_t address, uint8_t data){

}

void i2c_sendWord(I2C_Device* dev, uint16_t address, uint16_t data){

}

void i2c_receive(I2C_Device* dev, uint16_t address, uint8_t* data, uint8_t len){

}

void i2c_receiveByte(I2C_Device* dev, uint16_t address, uint8_t* buff){

}

void i2c_receiveWord(I2C_Device* dev, uint16_t address, uint8_t* data){

}
