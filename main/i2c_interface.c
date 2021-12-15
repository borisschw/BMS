

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "i2c_interface.h"
#include "max1726x.h"


int i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    int err = i2c_param_config(i2c_master_port, &conf);
    if (err) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

//
//int i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
//{
//    if (size == 0) {
//        return ESP_OK;
//    }
//
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//
//    i2c_master_write_byte(cmd, (MAX1726X_I2C_ADDR<< 1) | READ_BIT , ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, data_rd[0] , ACK_CHECK_EN);
//
//    if (size > 1) {
//        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
//    }
//    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
//
//    i2c_master_stop(cmd);
//    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    return ret;
//}

int i2c_master_write_slave(i2c_port_t i2c_num, uint8_t device_address, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


int i2c_master_write_read_device(i2c_port_t i2c_num, uint8_t device_address,
                                       const uint8_t* write_buffer, size_t write_size,
                                       uint16_t* read_buffer, size_t read_size,
                                       TickType_t ticks_to_wait)
{
    int err = ESP_OK;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
    	i2c_cmd_link_delete(handle);
    }

    err = i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
    	i2c_cmd_link_delete(handle);
    }

    err = i2c_master_write(handle, write_buffer, write_size, true);
    if (err != ESP_OK) {
    	i2c_cmd_link_delete(handle);
    }

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
    	i2c_cmd_link_delete(handle);
    }

    err = i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
    	i2c_cmd_link_delete(handle);
    }

    err = i2c_master_read(handle, (uint8_t *)read_buffer, read_size*2, I2C_MASTER_LAST_NACK);
    if (err != ESP_OK) {
    	i2c_cmd_link_delete(handle);
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(i2c_num, handle, ticks_to_wait);

    i2c_cmd_link_delete(handle);
    return err;
}









