#ifndef __I2C_H__
#define __I2C_H__

#include <stdint.h>

typedef struct {
    const char* dev_path; // eg. "/dev/i2c-0"
    int handle;
} i2c_handle_t;

int i2c_init(i2c_handle_t* i2c_handle);
int i2c_deinit(i2c_handle_t* i2c_handle);
int i2c_set_addr(i2c_handle_t* i2c_handle, int i2c_addr);

int i2c_read_reg(i2c_handle_t* i2c_handle, uint8_t reg, uint8_t* data, uint8_t len);
int i2c_write_reg(i2c_handle_t* i2c_handle, uint8_t reg, uint8_t* data, uint8_t len);

int i2c_read_reg_u8(i2c_handle_t* i2c_handle, uint8_t reg, uint8_t* data);
int i2c_write_reg_u8(i2c_handle_t* i2c_handle, uint8_t reg, uint8_t data);

int i2c_read_reg_u16(i2c_handle_t* i2c_handle, uint8_t reg, uint16_t* data);
int i2c_write_reg_u16(i2c_handle_t* i2c_handle, uint8_t reg, uint16_t data);

#endif 
