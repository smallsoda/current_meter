//
//

#ifndef STORAGE_H_
#define STORAGE_H_

#include <stdint.h>


typedef struct
{
	uint8_t empty_status; // 0x5A - not empty, OTHER - empty
	uint8_t mode;         // 0 - RAW, 1 - TEXT
	uint8_t period;       // 0 - 1ms, 1 - 10ms, 2 - 100ms
	uint32_t : 8;         // Reserved
	uint32_t : 32;        // Reserved
	uint32_t : 32;        // Reserved
	uint32_t : 32;        // Reserved
} storage_settings_t;

typedef struct
{
	uint32_t oa0;  // Calibration coefficient for the operational amplifier 0
	uint32_t oa1;  // Calibration coefficient for the operational amplifier 1
	uint32_t oa2;  // Calibration coefficient for the operational amplifier 2
	uint32_t : 32; // Reserved
} storage_calib_t;

typedef struct
{
	storage_settings_t settings; // 16b struct
	storage_calib_t    calib;  // 16b struct
} storage_t;


void storage_get_params(storage_t *params);
void storage_set_params(storage_t *params);
void storage_set_params_settings(storage_settings_t *settings);
void storage_set_params_calib(storage_calib_t *calib);
//
void storage_set_default(void);

#endif /* STORAGE_H_ */
