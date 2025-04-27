#include "i2c.h"

static int32_t status = 0;

void init_i2c0(gpio_num_t SCL_PIN, gpio_num_t SDA_PIN){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        
        .sda_io_num = SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,

        .scl_io_num = SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,

        .master.clk_speed = 400000
    };

    status = i2c_param_config(I2C_NUM_0, &conf);

    #if defined(DEBUG_I2C)
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "Param config ok");
        }else{
            ESP_LOGE(I2C_TAG, "ERROR PARAM CONFIGURATION");
        }
    #endif

    status = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    
    #if defined(DEBUG_I2C)
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "Install ok");
        }else{
            ESP_LOGE(I2C_TAG, "ERROR INSTALLING DRIVER");
        }
    #endif
}

uint8_t i2c0_send(uint16_t dev_addr, uint16_t reg_addr, uint8_t* data, uint32_t data_len){

    uint8_t reg_addr_buff[2] = {0};

    reg_addr_buff[1] = reg_addr & 0xFF;
    reg_addr_buff[0] = (reg_addr >> 8) & 0xFF;

    #if defined(DEBUG_I2C)
        ESP_LOGI(I2C_TAG, "Starting I2C send...");
    #endif

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    status = i2c_master_start(cmd);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", START_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_write_byte(cmd, dev_addr | I2C_MASTER_WRITE, ACK_EN);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", WRITE_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_write(cmd, reg_addr_buff, sizeof(reg_addr_buff), ACK_EN);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", WRITE_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_write(cmd, data, data_len, ACK_EN);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", WRITE_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_stop(cmd);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", STOP_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

     status = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100);
    #if defined(DEBUG_I2C)
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", COMMAND_BEGIN_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else if(status == ESP_FAIL){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_FAIL_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    i2c_cmd_link_delete(cmd);

    return 0;
}

uint8_t i2c0_receive(uint16_t dev_addr, uint16_t reg_addr, uint8_t* buff, uint32_t buff_len){

    uint8_t reg_addr_buff[2] = {0};

    reg_addr_buff[1] = reg_addr & 0xFF;
    reg_addr_buff[0] = (reg_addr >> 8) & 0xFF;

    #if defined(DEBUG_I2C)
        ESP_LOGI(I2C_TAG, "Starting I2C read...");
    #endif

    i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();
    
    status = i2c_master_start(cmd1);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", START_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_write_byte(cmd1, dev_addr | I2C_MASTER_WRITE, ACK_EN);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", WRITE_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_write(cmd1, reg_addr_buff, sizeof(reg_addr_buff), ACK_EN);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", WRITE_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_stop(cmd1);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", STOP_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_cmd_begin(I2C_NUM_0, cmd1, 100);
    #if defined(DEBUG_I2C)
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", COMMAND_BEGIN_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else if(status == ESP_FAIL){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_FAIL_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    i2c_cmd_link_delete(cmd1);
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();

    status = i2c_master_start(cmd2);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", STOP_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_write_byte(cmd2, dev_addr | I2C_MASTER_READ, ACK_EN);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", WRITE_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    if(buff_len > 1){
        status = i2c_master_read(cmd2, buff, buff_len - 1, ACK_DIS);
        #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
            if(status == ESP_OK){
                ESP_LOGI(I2C_TAG, "%s", READ_OK_STRING);
            }else if(status == ESP_ERR_INVALID_ARG){
                ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
            }else{
                ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
            }
        #endif
    }

    status = i2c_master_read_byte(cmd2, &buff[buff_len - 1], ACK_EN);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", READ_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    status = i2c_master_stop(cmd2);
    #if defined(DEBUG_I2C) && DEBUG_I2C_LEVEL == 0
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", STOP_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif
    
    status = i2c_master_cmd_begin(I2C_NUM_0, cmd2, 100);
    #if defined(DEBUG_I2C)
        if(status == ESP_OK){
            ESP_LOGI(I2C_TAG, "%s", COMMAND_BEGIN_OK_STRING);
        }else if(status == ESP_ERR_INVALID_ARG){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_INVALID_ARG_STRING);
        }else if(status == ESP_FAIL){
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_FAIL_STRING);
        }else{
            ESP_LOGE(I2C_TAG, "ERROR %s", ESP_ERR_STRING);
        }
    #endif

    i2c_cmd_link_delete(cmd2);

    return 0;
}
