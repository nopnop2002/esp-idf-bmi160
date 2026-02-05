#include <stdio.h>
#include "bmi160.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#define I2C_NUM I2C_NUM_0
#define I2C_TICKS_TO_WAIT 100 // Maximum ticks to wait before issuing a timeout.
#define __DEBUG__ 0

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t user_i2c_read(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t length) {
	uint8_t out_buf[1];
	out_buf[0] = regAddr;
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, out_buf, sizeof(out_buf), data, length, I2C_TICKS_TO_WAIT));

#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "regAddr=0x%x length=%d", regAddr, length);
	for (int i=0;i<length;i++) {
		ESP_LOGI(__FUNCTION__, "data[%d]=0x%x", i, data[i]);
	}
#endif

	return 0;
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param length Number of bytes to write
 * @param data Array of bytes to write
 * @return Status of operation (true = success)
 */
int8_t user_i2c_write(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t length) {
#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "regAddr=0x%x length=%d", regAddr, length);
	for (int i=0;i<length;i++) {
		ESP_LOGI(__FUNCTION__, "data[%d]=0x%x", i, data[i]);
	}
#endif

	uint8_t *out_buf;
	out_buf = malloc(length+1);
	if (out_buf == NULL) {
		ESP_LOGE(__FUNCTION__, "malloc fail");
		return 0;
	}
	out_buf[0] = regAddr;
	for(int i=0;i<length;i++) out_buf[i+1] = data[i];
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, out_buf, length+1, I2C_TICKS_TO_WAIT));
	free(out_buf);

	return 0;
}

void user_delay_ms(uint32_t period) {
#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "period=%" PRIu32, period);
#endif
	esp_rom_delay_us(period*1000);
}

/** Initialize i2c master driver.
 */
void interface_init(struct bmi160_dev *dev) {
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.i2c_port = I2C_NUM,
		.scl_io_num = CONFIG_GPIO_SCL,
		.sda_io_num = CONFIG_GPIO_SDA,
		.flags.enable_internal_pullup = true,
	};

	//i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = CONFIG_I2C_ADDR,
		.scl_speed_hz = 400000,
	};

	//i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

	//I2C address is 0x68 (if SDO-pin is gnd) or 0x69 (if SDO-pin is vddio).
	//dev->id = BMI160_I2C_ADDR;
	dev->id = CONFIG_I2C_ADDR;
	dev->intf = BMI160_I2C_INTF;
	dev->read = user_i2c_read;
	dev->write = user_i2c_write;
	dev->delay_ms = user_delay_ms;
}

