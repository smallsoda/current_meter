//
//

#ifndef SLIP_H_
#define SLIP_H_

#include <stdint.h>


uint16_t slip_packet(uint8_t **slip_data, const uint8_t *raw_data, uint16_t length);

#endif /* SLIP_H_ */
