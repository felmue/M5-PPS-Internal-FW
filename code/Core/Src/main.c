/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_comm.h"
#include "flash.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  VS_VIN,
	VS_VOUT,
	CS_VOUT,
  VINT_REF,
  VTEMP,
  ADC1_CHN_MAX,
} ADC1_CHN;

typedef enum {
  PSU_DISABLED,
  PSU_CV, 
	PSU_CC,
  PSU_STATUS_MAX
} PSU_STATUS;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS_DEFAULT (106>>1)
#define ADC_MAX 4095
#define VREFINT_CALIB (*((uint16_t*)0x1FFF75AA))
#define VTEMP_CALIB   (*((uint16_t*)0x1FFF75A8))
#define IOUT_MAX 5.25f
#define VOUT_MAX 30.0f
#define IOUT_ZERO_MAX 10000
#define LIMIT_VALUE_0_3 0.305f
#define LIMIT_VALUE_0_4 0.405f

#define VOUT_FLASH_DATA_SIZE 240
#define IOUT_FLASH_DATA_SIZE 640
#define IOUT_ZERO_FLASH_DATA_SIZE 8
#define VIN_FLASH_DATA_SIZE 240
#define IIN_FLASH_DATA_SIZE 400
#define IIN_ZERO_MAX_FLASH_DATA_SIZE 8
#define I2C_ADDRESS_FLASH_DATA_SIZE 8
#define FLASH_DATA_SIZE VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE + IIN_FLASH_DATA_SIZE + IIN_ZERO_MAX_FLASH_DATA_SIZE + I2C_ADDRESS_FLASH_DATA_SIZE
#define APPLICATION_ADDRESS     ((uint32_t)0x08001800) 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PSU_STATUS psu_sys_status = PSU_DISABLED;
uint8_t seq = 0;
uint16_t adc1_vals[ADC1_CHN_MAX];
float adc1_rvals[ADC1_CHN_MAX];
float adc1_cal_vals[ADC1_CHN_MAX];

float adc1_cal_coeff[ADC1_CHN_MAX] = { 15.70588f, // VS_VIN
    11.0f, // VS_VOUT
    2.0f, // CS_VOUT
    1.0, // Not used
    1.0, // Not used
    };

float vout_pwm_calib = 1960.79f;
float iout_pwm_calib = 13000.0f;  // new
float iout_pwm_calib_0_3 = 26000.0f;  // new
float iout_pwm_calib_0_2 = 40000.0f;  // new
float iout_pwm_calib_0_1 = 160000.0f;  // new
float iout_pwm_calib_0_0_1 = 800000.0f;  // new

const float vout_pwm_offset_default[60] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};

float vout_pwm_offset[60] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};

const float iout_pwm_offset_default[160] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};

float iout_pwm_offset[160] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};

const float vin_adc_offset_default[60] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};

float vin_adc_offset[60] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};

const float iin_adc_offset_default[100] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                           1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};

float iin_adc_offset[100] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                             1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};

uint32_t iout_zero_offset[2] = {4000, 0};

volatile float iin_zero_max[2] = {0.2f, 5.0f};

uint8_t i2c_address[8] = {I2C_ADDRESS_DEFAULT, 0, 0, 0, 0, 0, 0, 0};

uint64_t vout_pwm_calib_setup = 0;
uint64_t iout_pwm_calib_setup = 0;  // new
uint64_t vin_adc_calib_setup = 0;  // new
uint64_t iin_adc_calib_setup = 0;  // new

uint8_t reg_byte = 0, sys_stu = PSU_DISABLED, sys_mode = PSU_DISABLED;
float vout_set = 0.0f;
float iout_set = 0.0f;
uint32_t iout_zero_set = 0;
volatile float iout_set_cal = 0.0f;
volatile float limit_0_3_0_4 = LIMIT_VALUE_0_4;
volatile float iin_zero_set = 0;
volatile float iin_max_set = 0;
		
uint16_t i_pwm_test = 0;
uint16_t v_pwm_test = 0;

uint8_t psu_iap_set_value = 0;
uint8_t psu_soft_reset_value = 0;
uint8_t psu_i2c_address_value = 0;
uint8_t psu_cal_save_value = 0;
uint8_t psu_vout_cal_clear_value = 0;
uint8_t psu_iout_cal_clear_value = 0;
uint8_t psu_vin_cal_clear_value = 0;
uint8_t psu_iin_cal_clear_value = 0;

uint8_t flash_data[FLASH_DATA_SIZE] = {0};
volatile uint32_t jump_bootloader_timeout = 0;
uint8_t i2c_addr_change_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);

	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void init_chipid(void)
{
  uint32_t chipid_w0, chipid_w1, chipid_w2;

  chipid_w0 = HAL_GetUIDw0();
  chipid_w1 = HAL_GetUIDw1();
  chipid_w2 = HAL_GetUIDw2();
  // chipid_w0 = 0x55;
  // chipid_w1 = 0x56;
  // chipid_w2 = 0x58;

  set_i2c_reg(PSU_UID_W0_1, 4, (uint8_t *)&chipid_w0);  
  set_i2c_reg(PSU_UID_W1_1, 4, (uint8_t *)&chipid_w1);  
  set_i2c_reg(PSU_UID_W2_1, 4, (uint8_t *)&chipid_w2);  
}

void i2c_port_set_to_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = (GPIO_PIN_8 | GPIO_PIN_9);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    memcpy(flash_data, (uint8_t*)&vout_pwm_offset, VOUT_FLASH_DATA_SIZE);
    memcpy(flash_data + VOUT_FLASH_DATA_SIZE, (uint8_t*)&iout_pwm_offset, IOUT_FLASH_DATA_SIZE);
    memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE, (uint8_t*)&iout_zero_offset, IOUT_ZERO_FLASH_DATA_SIZE);
    memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE, 
    (uint8_t*)&vin_adc_offset, VIN_FLASH_DATA_SIZE);
    memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE, 
    (uint8_t*)&iin_adc_offset, IIN_FLASH_DATA_SIZE);
    memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE + IIN_FLASH_DATA_SIZE, 
    (uint8_t*)&iin_zero_max, IIN_ZERO_MAX_FLASH_DATA_SIZE);
    memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE + IIN_FLASH_DATA_SIZE + IIN_ZERO_MAX_FLASH_DATA_SIZE, 
    i2c_address, I2C_ADDRESS_FLASH_DATA_SIZE);
    // memcpy(datatmp_current, p_current, 4); 
    int8_t is_flash_write_success = -1;
    is_flash_write_success = writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
    while(!is_flash_write_success) {
      is_flash_write_success = writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
      HAL_Delay(100);
    }
  } else {
    memcpy((uint8_t*)&vout_pwm_offset, flash_data, VOUT_FLASH_DATA_SIZE);
    memcpy((uint8_t*)&iout_pwm_offset, flash_data + VOUT_FLASH_DATA_SIZE, IOUT_FLASH_DATA_SIZE);
    memcpy((uint8_t*)&iout_zero_offset, flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE, IOUT_ZERO_FLASH_DATA_SIZE);
    memcpy((uint8_t*)&vin_adc_offset, 
    flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE, 
    VIN_FLASH_DATA_SIZE);
    memcpy((uint8_t*)&iin_adc_offset, 
    flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE, 
    IIN_FLASH_DATA_SIZE);
    memcpy((uint8_t*)&iin_zero_max, 
    flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE + IIN_FLASH_DATA_SIZE, 
    IIN_ZERO_MAX_FLASH_DATA_SIZE);
    memcpy(i2c_address, 
    flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE + IIN_FLASH_DATA_SIZE + IIN_ZERO_MAX_FLASH_DATA_SIZE, 
    I2C_ADDRESS_FLASH_DATA_SIZE);
    // memcpy(p_voltage, datatmp_voltage, 4);
    // memcpy(p_current, datatmp_current, 4);    
  }
  psu_i2c_address_value = i2c_address[0];
}

void flash_data_write_back(void)
{
  memcpy(flash_data, (uint8_t*)&vout_pwm_offset, VOUT_FLASH_DATA_SIZE);
  memcpy(flash_data + VOUT_FLASH_DATA_SIZE, (uint8_t*)&iout_pwm_offset, IOUT_FLASH_DATA_SIZE);
  memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE, (uint8_t*)&iout_zero_offset, IOUT_ZERO_FLASH_DATA_SIZE);
  memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE, 
  (uint8_t*)&vin_adc_offset, VIN_FLASH_DATA_SIZE); 
  memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE, 
  (uint8_t*)&iin_adc_offset, IIN_FLASH_DATA_SIZE);
  memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE + IIN_FLASH_DATA_SIZE, 
  (uint8_t*)&iin_zero_max, IIN_ZERO_MAX_FLASH_DATA_SIZE);  
  memcpy(flash_data + VOUT_FLASH_DATA_SIZE + IOUT_FLASH_DATA_SIZE + IOUT_ZERO_FLASH_DATA_SIZE + VIN_FLASH_DATA_SIZE + IIN_FLASH_DATA_SIZE + IIN_ZERO_MAX_FLASH_DATA_SIZE, 
  i2c_address, I2C_ADDRESS_FLASH_DATA_SIZE);   
  psu_i2c_address_value = i2c_address[0];
  int8_t is_flash_write_success = -1;
  is_flash_write_success = writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  while(!is_flash_write_success) {
    is_flash_write_success = writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
    HAL_Delay(100);
  }
}

void set_vout_offset_to_default(void)
{
  memcpy(vout_pwm_offset, vout_pwm_offset_default, VOUT_FLASH_DATA_SIZE);
}

void set_iout_offset_to_default(void)
{
  memcpy(iout_pwm_offset, iout_pwm_offset_default, IOUT_FLASH_DATA_SIZE);
}

void set_vin_offset_to_default(void)
{
  memcpy(vin_adc_offset, vin_adc_offset_default, VIN_FLASH_DATA_SIZE);
}

void set_iin_offset_to_default(void)
{
  memcpy(iin_adc_offset, iin_adc_offset_default, IIN_FLASH_DATA_SIZE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  HAL_ADC_Stop_DMA(hadc);
  
  for (uint8_t i = 0; i < ADC1_CHN_MAX - 2; i++)
  {
    // Compute voltaga before calibration
    adc1_rvals[i] = (adc1_vals[i] * 3.3f) / 65535.0f;
    adc1_cal_vals[i] = adc1_rvals[i] * adc1_cal_coeff[i];
  }

  // Compute temperature
  uint16_t temp_adc_12bit = (float)adc1_vals[VTEMP]/65535.0f*4095;
  uint16_t int_adc_12bit = (float)adc1_vals[VINT_REF]/65535.0f*4095;
  adc1_cal_vals[VTEMP] = __HAL_ADC_CALC_TEMPERATURE(__HAL_ADC_CALC_VREFANALOG_VOLTAGE(int_adc_12bit, ADC_RESOLUTION_12B), temp_adc_12bit, ADC_RESOLUTION_12B);
  // Cout
  adc1_cal_vals[CS_VOUT] = adc1_cal_vals[CS_VOUT] - iin_zero_max[0];
  // int32_t iin_set_int = (int32_t)(adc1_cal_vals[CS_VOUT]*10);
  // if (iin_set_int < 0)
  //   iin_set_int = 0;
  // if (iin_set_int >= 50)
  //   iin_set_int = 49;  
  // float iin_set_cal = adc1_cal_vals[CS_VOUT] * iin_adc_offset[iin_set_int*2] + iin_adc_offset[iin_set_int*2+1]; 
  float iin_set_cal = adc1_cal_vals[CS_VOUT] * iin_adc_offset[0] + iin_adc_offset[1]; 

  // int32_t vin_set_int = (int32_t)adc1_cal_vals[VS_VOUT];
  // if (vin_set_int >= 30)
  //   vin_set_int = 29;
  // float vin_set_cal = adc1_cal_vals[VS_VOUT] * vin_adc_offset[vin_set_int*2] + vin_adc_offset[vin_set_int*2+1];  
  float vin_set_cal = adc1_cal_vals[VS_VOUT] * vin_adc_offset[0] + vin_adc_offset[1];  

	// update i2c data
  set_i2c_reg(PSU_VOUT_READBACK_1, 4, (uint8_t*) &vin_set_cal);
  set_i2c_reg(PSU_IOUT_READBACK_1, 4, (uint8_t*) &iin_set_cal);
  set_i2c_reg(PSU_VIN_READBACK_1, 4, (uint8_t*) &adc1_cal_vals[VS_VIN]);
  set_i2c_reg(PSU_TEMP_READBACK_1, 4, (uint8_t*) &adc1_cal_vals[VTEMP]);

  set_i2c_reg(PSU_DATA_FLAG, 1, &seq);
  seq++;

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_vals, ADC1_CHN_MAX);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  // MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  init_flash_data();

  user_i2c_init();
  init_i2c_comm();  
  // flash_data_write_back();

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_vals, ADC1_CHN_MAX);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	// voltage control
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  //TIM3->CCR2 = (uint16_t) (0.0 * vout_pwm_calib);
	
	// current control
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // TIM3->CCR1 = (uint16_t) (1.0 * iout_pwm_calib);
	// TIM3->CCR1 = 65535;

  // driver enable
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);  // A5

	// output enable
  HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);  // A12

  init_chipid();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if 1
		// check enable flag
    get_i2c_reg(MODULE_ENABLE, 1, &reg_byte);
    if (reg_byte != sys_stu)
    {
      sys_stu = reg_byte;
      if (sys_stu) // enable output
      {
        HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);  // A12
      } else { // disable output
        // shut up ahhhh
        HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);  // A12
        // turn off CV/CC LED
        HAL_GPIO_WritePin(USER_LED_R_GPIO_Port, USER_LED_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED_G_GPIO_Port, USER_LED_G_Pin, GPIO_PIN_RESET);
        // update running mode reg
        sys_mode = PSU_DISABLED;
        set_i2c_reg(PSU_RUNNING_MODE, 1, &sys_stu);
      }
    }
    // check mode when output enable
    if (sys_stu)
    {
      if (HAL_GPIO_ReadPin(CVCC_STU_GPIO_Port, CVCC_STU_Pin) == GPIO_PIN_RESET) // A1
      {
        sys_mode = PSU_CV;  // Red LED on
        HAL_GPIO_WritePin(USER_LED_R_GPIO_Port, USER_LED_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED_G_GPIO_Port, USER_LED_G_Pin, GPIO_PIN_SET);
      } else
      {
        sys_mode = PSU_CC;  // Green LED on
        HAL_GPIO_WritePin(USER_LED_R_GPIO_Port, USER_LED_R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(USER_LED_G_GPIO_Port, USER_LED_G_Pin, GPIO_PIN_RESET);        
      }

      // check running mode and update
      get_i2c_reg(PSU_RUNNING_MODE, 1, &reg_byte);
      if (sys_mode != reg_byte)
      {
        set_i2c_reg(PSU_RUNNING_MODE, 1, &sys_mode);
      }

      // check voltage and current setting
      float vout_set_read = 0.0f, iout_set_read = 0.0f;
      uint32_t iout_zero_read = 0;
      get_i2c_reg(PSU_VOUT_SET_1, 4, (uint8_t*) &vout_set_read);
      get_i2c_reg(PSU_IOUT_SET_1, 4, (uint8_t*) &iout_set_read);
      get_i2c_reg(PSU_IOUT_CALIB_ZERO_1, 4, (uint8_t*) &iout_zero_read);
			
			
      if ((vout_set_read > VOUT_MAX) || (vout_set_read < 0.0f))
      {
        vout_set = 0.0f;
      }

      if ((iout_set_read > IOUT_MAX) || (iout_set_read < 0.0f))
      {
        iout_set = 0.0f;
      }

      if (iout_zero_read > IOUT_ZERO_MAX)
      {
        iout_zero_read = IOUT_ZERO_MAX;
      }

      if (vout_set != vout_set_read)
      {
        int32_t vout_set_int = (int32_t)vout_set_read;
        if (vout_set_int >= 30)
          vout_set_int = 29;
        float vout_set_cal = vout_set_read * vout_pwm_offset[vout_set_int*2] + vout_pwm_offset[vout_set_int*2+1];
        if (vout_set_cal < 0.0f)
          vout_set_cal = 0.0f;
        TIM3->CCR2 = (uint16_t) (vout_set_cal * vout_pwm_calib);
        vout_set = vout_set_read;
      }

      if (iout_zero_set != iout_zero_read)
      {
        TIM3->CCR1 = (uint16_t) (iout_zero_read);
        iout_zero_set = iout_zero_read;
      }

      if (iout_set != iout_set_read)
      {
        if (iout_set_read >= 0.295f && iout_set_read <= limit_0_3_0_4) {
          iout_set_cal = iout_set_read * iout_pwm_offset[158] + iout_pwm_offset[159];
          uint16_t iout_pwm_temp = (uint16_t) (iout_set_cal * iout_pwm_calib_0_3);
          if (iout_pwm_temp < iout_zero_offset[0]) {
            iout_pwm_temp = iout_zero_offset[0];
          }
          TIM3->CCR1 = iout_pwm_temp;          
        }
        else if (iout_set_read >= 0.005f && iout_set_read <= 0.015f) {
          iout_set_cal = iout_set_read * iout_pwm_offset[100] + iout_pwm_offset[101];
          uint16_t iout_pwm_temp = (uint16_t) (iout_set_cal * iout_pwm_calib_0_0_1);
          if (iout_pwm_temp < iout_zero_offset[0]) {
            iout_pwm_temp = iout_zero_offset[0];
          }
          TIM3->CCR1 = iout_pwm_temp;           
        }        
        else if (iout_set_read >= 0.095f && iout_set_read <= 0.105f) {
          iout_set_cal = iout_set_read * iout_pwm_offset[118] + iout_pwm_offset[119];
          uint16_t iout_pwm_temp = (uint16_t) (iout_set_cal * iout_pwm_calib_0_1);
          if (iout_pwm_temp < iout_zero_offset[0]) {
            iout_pwm_temp = iout_zero_offset[0];
          }
          TIM3->CCR1 = iout_pwm_temp;          
        }        
        else if (iout_set_read >= 0.195f && iout_set_read <= 0.205f) {
          iout_set_cal = iout_set_read * iout_pwm_offset[138] + iout_pwm_offset[139];
          uint16_t iout_pwm_temp = (uint16_t) (iout_set_cal * iout_pwm_calib_0_2);
          if (iout_pwm_temp < iout_zero_offset[0]) {
            iout_pwm_temp = iout_zero_offset[0];
          }
          TIM3->CCR1 = iout_pwm_temp;          
        }        
        else if (iout_set_read <= 0.305f) {
          int32_t iout_set_int = (int32_t)((iout_set_read)*100);
          iout_set_int = iout_set_int;
          if (iout_set_int >= 30)
            iout_set_int = 29;    
            iout_set_cal = iout_set_read * iout_pwm_offset[iout_set_int*2+100] + iout_pwm_offset[iout_set_int*2+101];

          if ((iout_set_read > 0.2f) && (iout_set_read <= 0.3f)) {
            uint16_t iout_pwm_temp = (uint16_t) (iout_set_cal * iout_pwm_calib_0_3);
            if (iout_pwm_temp < iout_zero_offset[0]) {
              iout_pwm_temp = iout_zero_offset[0];
            }
            TIM3->CCR1 = iout_pwm_temp;
          }     
          else if ((iout_set_read > 0.1f) && (iout_set_read <= 0.2f)) {
            uint16_t iout_pwm_temp = (uint16_t) (iout_set_cal * iout_pwm_calib_0_2);
            if (iout_pwm_temp < iout_zero_offset[0]) {
              iout_pwm_temp = iout_zero_offset[0];
            }
            TIM3->CCR1 = iout_pwm_temp;          
          }     
          else if ((iout_set_read > 0.01f) && (iout_set_read <= 0.1f)) {
            uint16_t iout_pwm_temp = (uint16_t) (iout_set_cal * iout_pwm_calib_0_1);
            if (iout_pwm_temp < iout_zero_offset[0]) {
              iout_pwm_temp = iout_zero_offset[0];
            }
            TIM3->CCR1 = iout_pwm_temp;            
          }     
          else if (iout_set_read <= 0.01f) {
            uint16_t iout_pwm_temp = (uint16_t) (iout_set_cal * iout_pwm_calib_0_0_1);
            if (iout_pwm_temp < iout_zero_offset[0]) {
              iout_pwm_temp = iout_zero_offset[0];
            }
            TIM3->CCR1 = iout_pwm_temp;                      
          } 
        }
        else if (iout_set_read > 0.305f) {
          int32_t iout_set_int = (int32_t)(iout_set_read*10);
          if (iout_set_int >= 50)
            iout_set_int = 49;
          iout_set_cal = iout_set_read * iout_pwm_offset[iout_set_int*2] + iout_pwm_offset[iout_set_int*2+1]; 

          uint16_t iout_pwm_temp = (uint16_t) (iout_set_cal * iout_pwm_calib);
          if (iout_pwm_temp < iout_zero_offset[0]) {
            iout_pwm_temp = iout_zero_offset[0];
          }
          TIM3->CCR1 = iout_pwm_temp; 
        }   
        iout_set = iout_set_read;
      }
    }
    
    uint64_t vout_pwm_calib_read = 0, iout_pwm_calib_read = 0, vin_adc_calib_read = 0, iin_adc_calib_read = 0;
    get_i2c_reg(PSU_VOUT_CALIB_1, 8, (uint8_t*) &vout_pwm_calib_read);
    get_i2c_reg(PSU_IOUT_CALIB_1, 8, (uint8_t*) &iout_pwm_calib_read);
    get_i2c_reg(PSU_VIN_CALIB_1, 8, (uint8_t*) &vin_adc_calib_read);
    get_i2c_reg(PSU_IIN_CALIB_1, 8, (uint8_t*) &iin_adc_calib_read);
    if (vout_pwm_calib_setup != vout_pwm_calib_read)
    {
      vout_pwm_calib_setup = vout_pwm_calib_read;
      uint32_t index = (vout_pwm_calib_read & 0x00000000ffffffff);
      uint32_t tmp = ((vout_pwm_calib_read >> 32) & 0x00000000ffffffff);
      if (index < VOUT_FLASH_DATA_SIZE / 4)
        memcpy((uint8_t*)&vout_pwm_offset[index], (uint8_t*)&tmp, 4);
    }    
    if (iout_pwm_calib_setup != iout_pwm_calib_read)
    {
      iout_pwm_calib_setup = iout_pwm_calib_read;
      uint32_t index = (iout_pwm_calib_read & 0x00000000ffffffff);
      uint32_t tmp = ((iout_pwm_calib_read >> 32) & 0x00000000ffffffff);
      if (index < IOUT_FLASH_DATA_SIZE / 4)
        memcpy((uint8_t*)&iout_pwm_offset[index], (uint8_t*)&tmp, 4);
    } 
    if (vin_adc_calib_setup != vin_adc_calib_read)
    {
      vin_adc_calib_setup = vin_adc_calib_read;
      uint32_t index = (vin_adc_calib_read & 0x00000000ffffffff);
      uint32_t tmp = ((vin_adc_calib_read >> 32) & 0x00000000ffffffff);
      if (index < VIN_FLASH_DATA_SIZE / 4)
        memcpy((uint8_t*)&vin_adc_offset[index], (uint8_t*)&tmp, 4);
    } 
    if (iin_adc_calib_setup != iin_adc_calib_read)
    {
      iin_adc_calib_setup = iin_adc_calib_read;
      uint32_t index = (iin_adc_calib_read & 0x00000000ffffffff);
      uint32_t tmp = ((iin_adc_calib_read >> 32) & 0x00000000ffffffff);
      if (index < IIN_FLASH_DATA_SIZE / 4)
        memcpy((uint8_t*)&iin_adc_offset[index], (uint8_t*)&tmp, 4);
    } 

    uint8_t psu_iap_set_read = 0;
    get_i2c_reg(PSU_IAP_SET, 1, (uint8_t*) &psu_iap_set_read);
    if (psu_iap_set_value != psu_iap_set_read)
    {
      set_i2c_reg(PSU_IAP_SET, 1, (uint8_t*) &psu_iap_set_value);
      HAL_I2C_DeInit(&hi2c1);
      i2c_port_set_to_input();
      // driver disable
      HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);  // A5
      // output disable
      HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);  // A12      
      HAL_ADC_DeInit(&hadc1);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	    // voltage control
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	    // current control
	    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);      
      HAL_TIM_Base_MspDeInit(&htim3);
      HAL_TIM_Base_MspDeInit(&htim1);
      HAL_TIM_PWM_DeInit(&htim1);      
      HAL_TIM_PWM_DeInit(&htim3);
      __HAL_RCC_DMA1_CLK_DISABLE();
      HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);            
      while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
      {
        jump_bootloader_timeout++;
        if (jump_bootloader_timeout >= 60000) {
          break;
        }
      }
      if (jump_bootloader_timeout < 60000) {
        HAL_NVIC_SystemReset();
      } else {
        MX_GPIO_Init();
        MX_DMA_Init();
        MX_ADC1_Init();
        user_i2c_init();
        MX_TIM1_Init();
        MX_TIM3_Init();
        init_i2c_comm();
        init_flash_data();

        HAL_ADCEx_Calibration_Start(&hadc1);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_vals, ADC1_CHN_MAX);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

        // voltage control
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        //TIM3->CCR2 = (uint16_t) (0.0 * vout_pwm_calib);
        
        // current control
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        // TIM3->CCR1 = (uint16_t) (1.0 * iout_pwm_calib);
        // TIM3->CCR1 = 65535;

        // driver enable
        HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);  // A5

        // output enable
        HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);  // A12        
        jump_bootloader_timeout = 0;
      }      
    }

    uint8_t psu_soft_reset_read = 0;
    get_i2c_reg(PSU_SOFT_RESET, 1, (uint8_t*) &psu_soft_reset_read);
    if (psu_soft_reset_value != psu_soft_reset_read)
    {
      set_i2c_reg(PSU_SOFT_RESET, 1, (uint8_t*) &psu_soft_reset_value);
      HAL_NVIC_SystemReset();     
    }

    uint8_t psu_i2c_address_read = 0;
    get_i2c_reg(PSU_I2C_ADDRESS, 1, (uint8_t*) &psu_i2c_address_read);
    if (psu_i2c_address_value != psu_i2c_address_read)
    {
      if (psu_i2c_address_read > 0 && psu_i2c_address_read < 128) {
        i2c_address[0] = psu_i2c_address_read;
        flash_data_write_back();
        HAL_Delay(10);
        HAL_I2C_DeInit(&hi2c1);
        user_i2c_init();

        init_i2c_comm();
      }
    }

    uint8_t psu_cal_save_read = 0;
    get_i2c_reg(PSU_CAL_SAVE, 1, (uint8_t*) &psu_cal_save_read);
    if (psu_cal_save_value != psu_cal_save_read)
    {
      set_i2c_reg(PSU_CAL_SAVE, 1, (uint8_t*) &psu_cal_save_value);
      iout_zero_offset[0] = iout_zero_set;
      flash_data_write_back();  
      limit_0_3_0_4 = LIMIT_VALUE_0_4;    
    }

    uint8_t psu_iout_cal_clear_read = 0;
    get_i2c_reg(PSU_PSU_IOUT_CALIB_CLEAR, 1, (uint8_t*) &psu_iout_cal_clear_read);
    if (psu_iout_cal_clear_value != psu_iout_cal_clear_read)
    {
      set_i2c_reg(PSU_PSU_IOUT_CALIB_CLEAR, 1, (uint8_t*) &psu_iout_cal_clear_value);
      set_iout_offset_to_default();
      limit_0_3_0_4 = LIMIT_VALUE_0_3;
    }

    uint8_t psu_vout_cal_clear_read = 0;
    get_i2c_reg(PSU_PSU_VOUT_CALIB_CLEAR, 1, (uint8_t*) &psu_vout_cal_clear_read);
    if (psu_vout_cal_clear_value != psu_vout_cal_clear_read)
    {
      set_i2c_reg(PSU_PSU_VOUT_CALIB_CLEAR, 1, (uint8_t*) &psu_vout_cal_clear_value);
      set_vout_offset_to_default();
    }

    uint8_t psu_vin_cal_clear_read = 0;
    get_i2c_reg(PSU_PSU_VIN_CALIB_CLEAR, 1, (uint8_t*) &psu_vin_cal_clear_read);
    if (psu_vin_cal_clear_value != psu_vin_cal_clear_read)
    {
      set_i2c_reg(PSU_PSU_VIN_CALIB_CLEAR, 1, (uint8_t*) &psu_vin_cal_clear_value);
      set_vin_offset_to_default();
    }

    uint8_t psu_iin_cal_clear_read = 0;
    get_i2c_reg(PSU_PSU_IIN_CALIB_CLEAR, 1, (uint8_t*) &psu_iin_cal_clear_read);
    if (psu_iin_cal_clear_value != psu_iin_cal_clear_read)
    {
      set_i2c_reg(PSU_PSU_IIN_CALIB_CLEAR, 1, (uint8_t*) &psu_iin_cal_clear_value);
      set_iin_offset_to_default();
    }

    float iin_zero_read = 0.0f, iin_max_read = 0.0f;
    get_i2c_reg(PSU_IIN_CALIB_ZERO_1, 4, (uint8_t*) &iin_zero_read);
    get_i2c_reg(PSU_IIN_CALIB_MAX_1, 4, (uint8_t*) &iin_max_read);   
    if (iin_zero_set != iin_zero_read)
    {
      iin_zero_max[0] = iin_zero_read;
      iin_zero_set = iin_zero_read;
    }

    if (iin_max_set != iin_max_read)
    {
      iin_zero_max[1] = iin_max_read;
      iin_max_set = iin_max_read;
    }    
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
