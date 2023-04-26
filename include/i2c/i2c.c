#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "i2c.h"


int i2c_init(i2c_handle_t* i2c_handle)
{
	if ((i2c_handle->handle = open(i2c_handle->dev_path, O_RDWR)) < 0) {
		printf("Failed to open the I2C bus '%s'!\r\n", i2c_handle->dev_path);
		return -1;
	}
	return 0;
}

int i2c_deinit(i2c_handle_t* i2c_handle)
{
	if (i2c_handle->handle) {
		close(i2c_handle->handle);
		i2c_handle->handle = 0;
	}
	return 0;
}

int i2c_set_addr(i2c_handle_t* i2c_handle, int i2c_addr)
{
	if (!i2c_handle->handle) {
		return -1;
	}

	if (ioctl(i2c_handle->handle, I2C_SLAVE, i2c_addr) < 0) {
		printf("Failed to acquire I2C bus access and/or talk to slave.\r\n");
		return -2;
	}
	return 0;
}


int i2c_read_reg(i2c_handle_t* i2c_handle, uint8_t reg, uint8_t* data, uint8_t len)
{
	if (!i2c_handle->handle) {
		return -1;
	}
	
	if (write(i2c_handle->handle, &reg, sizeof(reg)) != sizeof(reg)){
		return -2;
	} else {
		if (read(i2c_handle->handle, data, len) != len){
			return -3; 
		}
	}
	return 0;
}

int i2c_write_reg(i2c_handle_t* i2c_handle, uint8_t reg, uint8_t* data, uint8_t len)
{
	if (!i2c_handle->handle) {
		return -1;
	}
	
	uint8_t buf[len+1];

	buf[0] = reg;
	for (uint8_t i = 0; i < len; i++) {
		buf[i+1] = data[i];
	}

	if (write(i2c_handle->handle, buf, len+1) != len+1){
		return -2;
	}
	return 0;
}

int i2c_read_reg_u8(i2c_handle_t* i2c_handle, uint8_t reg, uint8_t* data)
{
	return i2c_read_reg(i2c_handle, reg, data, 1);
}

int i2c_write_reg_u8(i2c_handle_t* i2c_handle, uint8_t reg, uint8_t data)
{
	return i2c_write_reg(i2c_handle, reg, &data, 1);
}

int i2c_read_reg_u16(i2c_handle_t* i2c_handle, uint8_t reg, uint16_t* data)
{
	if (i2c_read_reg(i2c_handle, reg, (uint8_t*) data, 2)) {
		return -1;
	}

	*data = (*data >> 8) | (*data << 8); // swap MSB and LSB
	
	return 0;
}

int i2c_write_reg_u16(i2c_handle_t* i2c_handle, uint8_t reg, uint16_t data)
{
	data = (data >> 8) | (data << 8); // swap MSB and LSB
	//printf("%02X %04X\n", reg, data);
	return i2c_write_reg(i2c_handle, reg, (uint8_t*) &data, 2);
}

