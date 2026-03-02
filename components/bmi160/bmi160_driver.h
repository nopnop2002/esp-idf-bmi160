#ifdef __cplusplus
extern "C" {
#endif
int8_t user_i2c_read(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t length);
int8_t user_i2c_write(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t length);
void user_delay_ms(uint32_t period);
void interface_init(struct bmi160_dev *dev);
#ifdef __cplusplus
}
#endif
