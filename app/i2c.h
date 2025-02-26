#ifndef __NORDIC_52832_I2C_
#define __NORDIC_52832_I2C_

#define DEFAULT_FLAG  0
#define SEND_I2C_DATA 1
#define READ_I2C_HEAD 2
#define READ_I2C_DATA 3

#define DATA_TYPE_NUS  1
#define DATA_TYPE_FIDO 2

bool i2c_master_write(uint8_t* buf, uint32_t len);
bool i2c_master_read(void);
int twi_master_init(void);
void get_i2c_data(uint8_t** data, uint16_t* len);
bool get_i2c_data_flag(void);
void set_i2c_data_flag(bool flag);
bool i2c_master_write_fido(uint8_t* buf, uint32_t len);
void twi_read_data(void);
#endif
