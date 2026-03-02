idf_component_register(
	SRCS "bmi160.c" "bmi160_driver.c"
	PRIV_REQUIRES driver esp_driver_i2c esp_timer
	INCLUDE_DIRS "."
)
