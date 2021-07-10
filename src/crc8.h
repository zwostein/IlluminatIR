#ifndef CRC8_INCLUDED
#define CRC8_INCLUDED

#include <stddef.h>
#include <stdint.h>

#define CRC8_INITIAL_SEED (0)

uint8_t crc8( const uint8_t * data, size_t data_size, uint8_t crc );

#endif
