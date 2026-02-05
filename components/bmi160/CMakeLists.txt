idf_component_register(SRCS "bmi160.c" "bmi160_driver.c"
                      INCLUDE_DIRS "."
                      PRIV_REQUIRES driver esp_timer)
