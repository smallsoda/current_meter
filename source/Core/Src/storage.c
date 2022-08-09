//
//

#include "storage.h"

#include "stm32f1xx_hal.h"
#include <string.h>


#define STORAGE_EEPROM_ADDRESS 0xA0
#define STORAGE_START_ADDRESS  0x00

extern I2C_HandleTypeDef hi2c1;


// @brief Get data from EEPROM
//
static void get_data(uint8_t address, uint8_t size, uint8_t *buffer)
{
	while(HAL_I2C_IsDeviceReady(&hi2c1, STORAGE_EEPROM_ADDRESS, 32, HAL_MAX_DELAY) != HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1, STORAGE_EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, buffer, size, HAL_MAX_DELAY);
}


// @brief Set data to EEPROM
//
static void set_data(uint8_t address, uint8_t size, uint8_t *buffer)
{
	while(size > 16)
	{
		while(HAL_I2C_IsDeviceReady(&hi2c1, STORAGE_EEPROM_ADDRESS, 32, HAL_MAX_DELAY) != HAL_OK);
		HAL_I2C_Mem_Write(&hi2c1, STORAGE_EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, buffer, 16, HAL_MAX_DELAY);
		buffer += 16;
		address += 16;
		size -= 16;
	}
	while(HAL_I2C_IsDeviceReady(&hi2c1, STORAGE_EEPROM_ADDRESS, 32, HAL_MAX_DELAY) != HAL_OK);
	HAL_I2C_Mem_Write(&hi2c1, STORAGE_EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, buffer, size, HAL_MAX_DELAY);
}


// @brief Get parameters from storage
//
void storage_get_params(storage_t *params)
{
	get_data(STORAGE_START_ADDRESS, sizeof(storage_t), (uint8_t *) params);
}


// @brief Save parameters to storage
//
void storage_set_params(storage_t *params)
{
	set_data(STORAGE_START_ADDRESS, sizeof(storage_t), (uint8_t *) params);
}


// @brief Save settings parameters to storage
//
void storage_set_params_settings(storage_settings_t *settings)
{
	set_data(STORAGE_START_ADDRESS, sizeof(storage_settings_t), (uint8_t *) settings);
}


// @brief Save app parameters to storage
//
void storage_set_params_calib(storage_calib_t *calib)
{
	set_data(STORAGE_START_ADDRESS + sizeof(storage_settings_t), sizeof(storage_calib_t), (uint8_t *) calib);
}


// @brief Save default parameters to storage
//
void storage_set_default(void)
{
	storage_t params = {0};

	params.settings.empty_status = 0x5A;
	params.settings.mode = 0;
	params.settings.period = 2;
	params.calib.oa0 = 0;
	params.calib.oa1 = 0;
	params.calib.oa2 = 0;

	storage_set_params(&params);
}
