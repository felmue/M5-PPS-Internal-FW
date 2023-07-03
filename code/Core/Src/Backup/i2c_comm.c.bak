#include "stdint.h"
#include "string.h"

#include "stm32g030xx.h"
#include "i2c.h"

#include "i2c_comm.h"

uint8_t i2c_reg_file[I2C_REG_MAP_MAX] __attribute__((aligned(64))) = { 0 };
uint8_t i2c_reg_file_buffer[I2C_REG_MAP_MAX] __attribute__((aligned(64)))
    = { 0 };

I2C_STATE i2c_state = I2C_STATE_IDLE;
uint8_t i2c_reg_addr = 0;
uint8_t i2c_rx_buf = 0;

void init_i2c_comm()
{
  HAL_I2C_EnableListen_IT(&hi2c1);
  i2c_state = I2C_STATE_IDLE;

  i2c_reg_file[MODULE_ID_L] = 0x41;
  i2c_reg_file[MODULE_ID_H] = 0x10;
}

void set_i2c_reg(I2C_REG_MAP reg_addr, uint8_t len, uint8_t *data)
{
  memcpy(&i2c_reg_file[reg_addr], data, len);
}

void get_i2c_reg(I2C_REG_MAP reg_addr, uint8_t len, uint8_t *data)
{
  memcpy(data, &i2c_reg_file[reg_addr], len);
}

uint8_t* get_i2c_reg_addr(I2C_REG_MAP reg_addr)
{
  return &i2c_reg_file[reg_addr];
}

void (*_error_handler)();
void I2C_Error_Handler()
{
  HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();

  init_i2c_comm();
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t direction,
    uint16_t addrMatchCode)
{
  switch (direction) {
  case I2C_DIRECTION_TRANSMIT:
    i2c_state = I2C_STATE_RD_ADDR;
    if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx_buf, 1, I2C_FIRST_FRAME)
        != HAL_OK)
    {
      I2C_Error_Handler();
    }
    break;

  case I2C_DIRECTION_RECEIVE:
    i2c_state = I2C_STATE_WR_DATA;
    i2c_reg_addr = i2c_rx_buf;

    if (i2c_reg_addr >= I2C_REG_MAP_MAX)
    {
      i2c_reg_addr = I2C_REG_ZERO;
    }

    memcpy(&i2c_reg_file_buffer[i2c_reg_addr], &i2c_reg_file[i2c_reg_addr],
        I2C_REG_MAP_MAX - i2c_reg_addr);

    if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &i2c_reg_file_buffer[i2c_reg_addr],
        I2C_REG_MAP_MAX - i2c_reg_addr, I2C_NEXT_FRAME) != HAL_OK)
    {
      I2C_Error_Handler();
    }

    if (i2c_reg_addr++ >= I2C_REG_MAP_MAX)
    {
      i2c_reg_addr = I2C_REG_ZERO;
    }
    break;

  default:
    I2C_Error_Handler();
    break;
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  switch (i2c_state) {
  case I2C_STATE_WR_DATA:
    if (i2c_reg_addr >= I2C_REG_MAP_MAX)
    {
      i2c_reg_addr = I2C_REG_ZERO;
    }

    memcpy(&i2c_reg_file_buffer[i2c_reg_addr], &i2c_reg_file[i2c_reg_addr],
        I2C_REG_MAP_MAX - i2c_reg_addr);

    if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &i2c_reg_file_buffer[i2c_reg_addr],
        I2C_REG_MAP_MAX - i2c_reg_addr, I2C_NEXT_FRAME) != HAL_OK)
    {
      I2C_Error_Handler();
    }

    if (i2c_reg_addr++ >= I2C_REG_MAP_MAX)
    {
      i2c_reg_addr = I2C_REG_ZERO;
    }
    break;
  default:
    I2C_Error_Handler();
    break;
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  switch (i2c_state) {
  case I2C_STATE_RD_ADDR:
    i2c_reg_addr = i2c_rx_buf;
    i2c_state = I2C_STATE_RD_DATA;

    if (i2c_reg_addr >= I2C_REG_MAP_MAX)
    {
      i2c_reg_addr = I2C_REG_ZERO;
    }

    if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_reg_file[i2c_reg_addr], 1,
        I2C_FIRST_FRAME) != HAL_OK)
    {
      I2C_Error_Handler();
    }

    if (i2c_reg_addr++ >= I2C_REG_MAP_MAX)
    {
      i2c_reg_addr = I2C_REG_ZERO;
    }
    break;

  case I2C_STATE_RD_DATA:
    if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_reg_file[i2c_reg_addr],
        I2C_REG_MAP_MAX - i2c_reg_addr, I2C_FIRST_FRAME) != HAL_OK)
    {
      I2C_Error_Handler();
    }

    if (i2c_reg_addr++ >= I2C_REG_MAP_MAX)
    {
      i2c_reg_addr = I2C_REG_ZERO;
    }
    break;

  default:
    I2C_Error_Handler();
    break;
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
  {
    I2C_Error_Handler();
  } else
  {
    i2c_state = I2C_STATE_IDLE;
  }
}
