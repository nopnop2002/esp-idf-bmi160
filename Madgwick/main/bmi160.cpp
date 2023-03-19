/* The example of ESP-IDF
 *
 * This sample code is in the public domain.
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "cJSON.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "MPU";

// bmi160 stuff
#include "bmi160.h"
#include "bmi160_defs.h"
#define I2C_NUM I2C_NUM_0
//#define __DEBUG__ 1

// Source: https://github.com/arduino-libraries/MadgwickAHRS
#include "MadgwickAHRS.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

Madgwick madgwick;

/* IMU Data */
struct bmi160_dev sensor;
#if 0
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
#endif

// Accel & Gyro scale factor
float accel_sensitivity;
float gyro_sensitivity;

#if 0
double roll, pitch; // Roll and pitch are calculated using the accelerometer

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
#endif


/** Select an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 */
void SelectRegister(uint8_t devAddr, uint8_t regAddr){
	ESP_LOGD(__FUNCTION__, "devAddr=0x%x regAddr=0x%x", devAddr, regAddr);
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
}


/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t user_i2c_read(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t length) {
	i2c_cmd_handle_t cmd;
	SelectRegister(devAddr, regAddr);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, 1));

	if(length>1)
		ESP_ERROR_CHECK(i2c_master_read(cmd, data, length-1, I2C_MASTER_ACK));

	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+length-1, I2C_MASTER_NACK));

	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

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

	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	if(length>1)
		ESP_ERROR_CHECK(i2c_master_write(cmd, data, length-1, 0));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[length-1], 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

	return 0;
}

void user_delay_ms(uint32_t period) {
#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "period=%" PRIu32, period);
#endif
	esp_rom_delay_us(period*1000);
}

// Get time in seconds since boot
// Compatible with ROS's time.toSec() function
double TimeToSec() {
	int64_t _time = esp_timer_get_time(); // Get time in microseconds since boot
	double __time = (double)_time / 1000000;
	return __time;
}

void bmi160(void *pvParameters)
{
	//struct bmi160_dev sensor;
	//I2C address is 0x68 (if SDO-pin is gnd) or 0x69 (if SDO-pin is vddio).
	//sensor.id = BMI160_I2C_ADDR;
	sensor.id = CONFIG_I2C_ADDR;
	sensor.intf = BMI160_I2C_INTF;
	sensor.read = user_i2c_read;
	sensor.write = user_i2c_write;
	sensor.delay_ms = user_delay_ms;
	int8_t ret = bmi160_init(&sensor);
	if (ret == BMI160_OK)
	{
		ESP_LOGI(TAG, "BMI160 initialization success !");
		ESP_LOGI(TAG, "Chip ID 0x%X", sensor.chip_id);
	}
	else
	{
		ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
		vTaskDelete(NULL);
	}

	// Config Accel
	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G; // -2 --> +2[g]
	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
	accel_sensitivity = 16384.0; // g

	// Config Gyro
	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
	//sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS; // -250 --> +250[Deg/Sec]
	sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
	gyro_sensitivity = 131.2; // Deg/Sec

	ret = bmi160_set_sens_conf(&sensor);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 set_sens_conf fail %d", ret);
		vTaskDelete(NULL);
	}
	ESP_LOGI(TAG, "bmi160_set_sens_conf");

	double last_time_ = TimeToSec();
	int elasped = 0;

	bool initialized = false;
	float initial_roll = 0.0;
	float initial_pitch = 0.0;
	float initial_yaw = 0.0;

	// It takes time for the estimated value to stabilize.
	// It need about 4Sec.
	int initial_period = 400;

	while(1) {
		struct bmi160_sensor_data accel;
		struct bmi160_sensor_data gyro;
		int8_t ret = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);
		if (ret != BMI160_OK) {
			ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
			vTaskDelete(NULL);
		}
		ESP_LOGD(TAG, "accel=%d %d %d gyro=%d %d %d", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);

		// Get the elapsed time from the previous
		float dt = (TimeToSec() - last_time_);
		ESP_LOGD(TAG, "dt=%f",dt);
		last_time_ = TimeToSec();

		// Get Euler
		float ax = (float)accel.x / accel_sensitivity;
		float ay = (float)accel.y / accel_sensitivity;
		float az = (float)accel.z / accel_sensitivity; 
		float gx = (float)gyro.x / gyro_sensitivity;
		float gy = (float)gyro.y / gyro_sensitivity;
		float gz = (float)gyro.z / gyro_sensitivity;
		madgwick.updateIMU(gx, gy, gz, ax, ay, az, dt);
		float roll = madgwick.getRoll();
		float pitch = madgwick.getPitch();
		float yaw = madgwick.getYaw();
		ESP_LOGD(TAG, "roll=%f pitch=%f yaw=%f", roll, pitch, yaw);


		/* Print Data every 10 times */
		if (elasped > initial_period) {
			// Set the first data
			if (!initialized) {
				initial_roll = roll;
				initial_pitch = pitch;
				initial_yaw = yaw;
				initialized = true;
				initial_period = 10;
			}

#if 0
			printf("roll:%f", roll); printf(" ");
			printf("initial_roll:%f", initial_roll); printf(" ");
			printf("roll-initial_roll:%f", roll-initial_roll); printf(" ");
			printf("gyroXangle:%f", gyroXangle); printf(" ");
			printf("compAngleX:%f", compAngleX); printf(" ");
			printf("kalAngleX:%f", kalAngleX); printf(" ");
			printf("\n");

			printf("pitch: %f", pitch); printf(" ");
			printf("initial_pitch: %f", initial_pitch); printf(" ");
			printf("pitch-initial_pitch: %f", pitch-initial_pitch); printf(" ");
			printf("gyroYangle:%f", gyroYangle); printf(" ");
			printf("compAngleY:%f", compAngleY); printf(" ");
			printf("kalAngleY:%f", kalAngleY); printf("\t");
			printf("\n");
#endif

			// Send UDP packet
			float _roll = roll-initial_roll;
			float _pitch = pitch-initial_pitch;
			float _yaw = yaw-initial_yaw;
			ESP_LOGD(TAG, "roll:%f pitch=%f yaw=%f", roll, pitch, yaw);
			ESP_LOGI(TAG, "roll:%f pitch=%f yaw=%f", _roll, _pitch, _yaw);

			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = 0.0;
			if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
			}

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", _roll);
			cJSON_AddNumberToObject(request, "pitch", _pitch);
			cJSON_AddNumberToObject(request, "yaw", 0.0);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);

			vTaskDelay(1);
			elasped = 0;
		}
	
		elasped++;
		vTaskDelay(1);
	} // end while

	// Never reach here
	vTaskDelete( NULL );
}
