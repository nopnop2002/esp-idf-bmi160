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
#include "cJSON.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// bmi160 stuff
#include "bmi160.h"
#include "bmi160_defs.h"
#include "bmi160_driver.h"

// Source: https://github.com/arduino-libraries/MadgwickAHRS
#include "MadgwickAHRS.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

// Create the Madgwick instances
Madgwick madgwick;

/* IMU Data */
struct bmi160_dev sensor;

// Accel & Gyro scale factor
float accel_sensitivity;
float gyro_sensitivity;

/* Obtain offsets */
void obtain_offsets(struct bmi160_dev *dev, struct bmi160_offsets *offsets)
{
	struct bmi160_sensor_data accel;
	struct bmi160_sensor_data gyro;
	int32_t sum[6] = {0};
	for (int i=0;i<100;i++) {
		int8_t ret = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, dev);
		if (ret != BMI160_OK) {
			ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
			return;
		}
		//printf("accel=%d %d %d gyro=%d %d %d\n", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
		sum[0] += accel.x;
		sum[1] += accel.y;
		sum[2] += accel.z -16384;
		sum[3] += gyro.x;
		sum[4] += gyro.y;
		sum[5] += gyro.z;
		vTaskDelay(1);
	}
	float fsum[6];
	for (int i = 0; i < 6; ++i) {
		fsum[i] = sum[i] / 100.0f;
	}
	//printf("fsum accel=%f %f %f gyro=%f %f %f\n", fsum[0], fsum[1], fsum[2], fsum[3], fsum[4], fsum[5]);

	/* offset values set by user as per their reference 
	 * Resolution of accel = 3.9mg/LSB 
	 * Resolution of gyro  = (0.061degrees/second)/LSB */
	fsum[0] = fsum[0] / accel_sensitivity / 3.9 * 1000;
	fsum[1] = fsum[1] / accel_sensitivity / 3.9 * 1000;
	fsum[2] = fsum[2] / accel_sensitivity / 3.9 * 1000;
	fsum[3] = fsum[3] / gyro_sensitivity / 0.061;
	fsum[4] = fsum[4] / gyro_sensitivity / 0.061;
	fsum[5] = fsum[5] / gyro_sensitivity / 0.061;
	printf("offsets accel=%f %f %f gyro=%f %f %f\n", fsum[0], fsum[1], fsum[2], fsum[3], fsum[4], fsum[5]);

	offsets->off_acc_x = 0-(int)fsum[0];
	offsets->off_acc_y = 0-(int)fsum[1];
	offsets->off_acc_z = 0-(int)fsum[2];
	offsets->off_gyro_x = 0-(int)fsum[3];
	offsets->off_gyro_y = 0-(int)fsum[4];
	offsets->off_gyro_z = 0-(int)fsum[5];
}

/* Updating manual offsets to sensor */
int8_t write_offsets(struct bmi160_dev *dev, struct bmi160_offsets offsets)
{
	int8_t rslt = 0;
	/* FOC configuration structure */
	struct bmi160_foc_conf foc_conf;
	
	/* Enable offset update for accel */
	foc_conf.acc_off_en = BMI160_ENABLE;

	/* Enable offset update for gyro */
	foc_conf.gyro_off_en = BMI160_ENABLE;
	
	rslt = bmi160_set_offsets(&foc_conf, &offsets, dev);
	//printf("bmi160_set_offsets rslt=%d\n", rslt);
	
	/* After offset setting the data read from the 
	 * sensor will have the corresponding offset */
	
	return rslt;
}

// Get scaled value
void getMotion6(double *_ax, double *_ay, double *_az, double *_gx, double *_gy, double *_gz) {
	struct bmi160_sensor_data accel;
	struct bmi160_sensor_data gyro;
	int8_t ret = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
		vTaskDelete(NULL);
	}
	ESP_LOGD(TAG, "accel=%d %d %d gyro=%d %d %d", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);

	// Convert relative to absolute
	*_ax = (double)accel.x / accel_sensitivity;
	*_ay = (double)accel.y / accel_sensitivity;
	*_az = (double)accel.z / accel_sensitivity;
	*_gx = (double)gyro.x / gyro_sensitivity;
	*_gy = (double)gyro.y / gyro_sensitivity;
	*_gz = (double)gyro.z / gyro_sensitivity;
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
	// Initialize i2c
	interface_init(&sensor);

	// Initialize IMU
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

	// Config sensor
	ret = bmi160_set_sens_conf(&sensor);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 set_sens_conf fail %d", ret);
		vTaskDelete(NULL);
	}
	ESP_LOGI(TAG, "bmi160_set_sens_conf");

	// Calcurate offsets
	ESP_LOGW(TAG, "IMU is currently being calibrated. Please do not move it.");
	struct bmi160_offsets offsets;
	obtain_offsets(&sensor, &offsets);

	// Write offsets
	ret = write_offsets(&sensor, offsets);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 write_offsets fail %d", ret);
		vTaskDelete(NULL);
	}
	vTaskDelay(10); // Wait for write
	ESP_LOGW(TAG, "IMU calibration is complete.");

	double ax, ay, az;
	double gx, gy, gz;
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
		getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//printf("%f %f %f - %f %f %f\n", ax, ay, az, gx, gy, gz);
		
		// Get the elapsed time from the previous
		float dt = (TimeToSec() - last_time_);
		ESP_LOGD(TAG, "dt=%f",dt);
		last_time_ = TimeToSec();

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
			printf("\n");

			printf("pitch: %f", pitch); printf(" ");
			printf("initial_pitch: %f", initial_pitch); printf(" ");
			printf("pitch-initial_pitch: %f", pitch-initial_pitch); printf(" ");
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
