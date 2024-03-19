#ifndef __I2C_PROTOCOL_H
#define __I2C_PROTOCOL_H

typedef enum {
    MODULE_ID_L = 0x00, // 2 bytes
    MODULE_ID_H = 0x01, // 2 bytes

    MODULE_HW_VERSION = 0x02, // 1 byte
    MODULE_SW_VERSION = 0x03, // 1 byte

    MODULE_ENABLE = 0x04, // 1 byte

    PSU_RUNNING_MODE = 0x05, // 1 byte

    DUMMY_BYTE = 0x06,

    PSU_DATA_FLAG = 0x07, // 1 byte

    PSU_VOUT_READBACK_1 = 0x08, // 4 bytes, float
    PSU_VOUT_READBACK_2,
    PSU_VOUT_READBACK_3,
    PSU_VOUT_READBACK_4,

    PSU_IOUT_READBACK_1 = 0x0C, // 4 bytes, float
    PSU_IOUT_READBACK_2,
    PSU_IOUT_READBACK_3,
    PSU_IOUT_READBACK_4,

    PSU_TEMP_READBACK_1 = 0x10, // 4 bytes, float
    PSU_TEMP_READBACK_2,
    PSU_TEMP_READBACK_3,
    PSU_TEMP_READBACK_4,

    PSU_VIN_READBACK_1 = 0x14, // 4 bytes, float
    PSU_VIN_READBACK_2,
    PSU_VIN_READBACK_3,
    PSU_VIN_READBACK_4,

    PSU_VOUT_SET_1 = 0x18, // 4 bytes, float
    PSU_VOUT_SET_2,
    PSU_VOUT_SET_3,
    PSU_VOUT_SET_4,

    PSU_IOUT_SET_1 = 0x1C, // 4 bytes, float
    PSU_IOUT_SET_2,
    PSU_IOUT_SET_3,
    PSU_IOUT_SET_4,

    PSU_VOUT_CALIB_1 = 0x20, // 4 bytes, float
    PSU_VOUT_CALIB_2,
    PSU_VOUT_CALIB_3,
    PSU_VOUT_CALIB_4,

    PSU_VOUT_CALIB_INDEX_1 = 0x24, // 4 bytes, float
    PSU_VOUT_CALIB_INDEX_2,
    PSU_VOUT_CALIB_INDEX_3,
    PSU_VOUT_CALIB_INDEX_4,

    PSU_IOUT_CALIB_1 = 0x28, // 4 bytes, float
    PSU_IOUT_CALIB_2,
    PSU_IOUT_CALIB_3,
    PSU_IOUT_CALIB_4,

    PSU_IOUT_CALIB_INDEX_1 = 0x2C, // 4 bytes, float
    PSU_IOUT_CALIB_INDEX_2,
    PSU_IOUT_CALIB_INDEX_3,
    PSU_IOUT_CALIB_INDEX_4,    

    PSU_IAP_SET = 0x30, // 1 bytes

    PSU_CAL_SAVE = 0x31, // 1 bytes

    PSU_PSU_VOUT_CALIB_CLEAR = 0x32, // 1 bytes

    PSU_PSU_IOUT_CALIB_CLEAR = 0x33, // 1 bytes

    PSU_IOUT_CALIB_ZERO_1 = 0x34, // 4 bytes, uint32_t
    PSU_IOUT_CALIB_ZERO_2,
    PSU_IOUT_CALIB_ZERO_3,
    PSU_IOUT_CALIB_ZERO_4,   

    PSU_VIN_CALIB_1 = 0x38, // 4 bytes, float
    PSU_VIN_CALIB_2,
    PSU_VIN_CALIB_3,
    PSU_VIN_CALIB_4,

    PSU_VIN_CALIB_INDEX_1 = 0x3C, // 4 bytes, float
    PSU_VIN_CALIB_INDEX_2,
    PSU_VIN_CALIB_INDEX_3,
    PSU_VIN_CALIB_INDEX_4,

    PSU_IIN_CALIB_1 = 0x40, // 4 bytes, float
    PSU_IIN_CALIB_2,
    PSU_IIN_CALIB_3,
    PSU_IIN_CALIB_4,

    PSU_IIN_CALIB_INDEX_1 = 0x44, // 4 bytes, float
    PSU_IIN_CALIB_INDEX_2,
    PSU_IIN_CALIB_INDEX_3,
    PSU_IIN_CALIB_INDEX_4, 

    PSU_PSU_VIN_CALIB_CLEAR = 0x48, // 1 bytes

    PSU_PSU_IIN_CALIB_CLEAR = 0x49, // 1 bytes  

    // PSU_I2C_MAP_DUMP_1 = 0x4A, // 6 bytes, dump
    // PSU_I2C_MAP_DUMP_2,
    // PSU_I2C_MAP_DUMP_3,
    // PSU_I2C_MAP_DUMP_4,              
    // PSU_I2C_MAP_DUMP_5,              
    // PSU_I2C_MAP_DUMP_6,              

    PSU_IIN_CALIB_ZERO_1 = 0x4A, // 4 bytes, float
    PSU_IIN_CALIB_ZERO_2,
    PSU_IIN_CALIB_ZERO_3,
    PSU_IIN_CALIB_ZERO_4,   

    PSU_IIN_CALIB_MAX_1 = 0x4E, // 4 bytes, float
    PSU_IIN_CALIB_MAX_2,
    PSU_IIN_CALIB_MAX_3,
    PSU_IIN_CALIB_MAX_4,   

    PSU_UID_W0_1 = 0x52, // 4 bytes, float
    PSU_UID_W0_2,
    PSU_UID_W0_3,
    PSU_UID_W0_4,   

    PSU_UID_W1_1 = 0x56, // 4 bytes, float
    PSU_UID_W1_2,
    PSU_UID_W1_3,
    PSU_UID_W1_4,   

    PSU_UID_W2_1 = 0x5A, // 4 bytes, float
    PSU_UID_W2_2,
    PSU_UID_W2_3,
    PSU_UID_W2_4,   

    PSU_SOFT_RESET = 0x5E, 

    PSU_I2C_ADDRESS = 0x5F, 

    I2C_REG_ZERO,

    I2C_REG_MAP_MAX
} I2C_REG_MAP;

typedef enum {
    I2C_STATE_IDLE = 0,
    I2C_STATE_STOP,
    I2C_STATE_RD_ADDR,
    I2C_STATE_RD_DATA,
    I2C_STATE_WR_DATA,
} I2C_STATE;


void init_i2c_comm();
void set_i2c_reg(I2C_REG_MAP reg_addr, uint8_t len, uint8_t *data);
void get_i2c_reg(I2C_REG_MAP reg_addr, uint8_t len, uint8_t *data);
uint8_t *get_i2c_reg_addr(I2C_REG_MAP reg_addr);
#endif
