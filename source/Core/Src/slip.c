//
//

#include "slip.h"

#include "stdlib.h"


// @brief Create SLIP packet
//
uint16_t slip_packet(uint8_t **slip_data, const uint8_t *raw_data, uint16_t length)
{
	uint16_t slip_length = length + 2;
	uint8_t *slip = malloc(slip_length);

	slip[0] = 0xC0;
	for(uint16_t i = 0, j = 1; i < length; i++, j++)
	{
		if(raw_data[i] == 0xC0)
		{
			slip_length++;
			slip = realloc(slip, slip_length);
			slip[j] = 0xDB;
			j++;
			slip[j] = 0xDC;
		}
		else if(raw_data[i] == 0xDB)
		{
			slip_length++;
			slip = realloc(slip, slip_length);
			slip[j] = 0xDB;
			j++;
			slip[j] = 0xDD;
		}
		else
		{
			slip[j] = raw_data[i];
		}
	}
	slip[slip_length - 1] = 0xC0;

	*slip_data = slip;
	return slip_length;
}
