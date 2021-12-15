
#ifndef I2C_INTERFACE_H_
#define I2C_INTERFACE_H_

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"


#define I2C_MASTER_TIMEOUT_MS 100
#define I2C_MASTER_SCL_IO 14               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 13					              /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 0 					 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000       	 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


int i2c_master_init(void);
int i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);


/**
 * @brief Perform a write followed by a read to a device on the I2C bus.
 *        A repeated start signal is used between the `write` and `read`, thus, the bus is
 *        not released until the two transactions are finished.
 *        This function is a wrapper to `i2c_master_start()`, `i2c_master_write()`, `i2c_master_read()`, etc...
 *        It shall only be called in I2C master mode.
 *
 * @param i2c_num I2C port number to perform the transfer on
 * @param device_address I2C device's 7-bit address
 * @param write_buffer Bytes to send on the bus
 * @param write_size Size, in bytes, of the write buffer
 * @param read_buffer Buffer to store the bytes received on the bus
 * @param read_size Size, in bytes, of the read buffer
 * @param ticks_to_wait Maximum ticks to wait before issuing a timeout.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave hasn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
int i2c_master_write_read_device(i2c_port_t i2c_num, uint8_t device_address,
                                       const uint8_t* write_buffer, size_t write_size,
                                       uint16_t* read_buffer, size_t read_size,
                                       TickType_t ticks_to_wait);

int i2c_master_write_slave(i2c_port_t i2c_num, uint8_t device_address, uint8_t *data_wr, size_t size);


#endif
