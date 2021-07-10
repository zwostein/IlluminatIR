#ifndef ILLUMINATIR_INCLUDED
#define ILLUMINATIR_INCLUDED

#include <stdint.h>


#define ILLUMINATIR_MIN_PACKET_SIZE 4
#define ILLUMINATIR_MAX_PACKET_SIZE 24

#define ILLUMINATIR_MIN_COBSPACKET_SIZE (ILLUMINATIR_MIN_PACKET_SIZE + 1)
#define ILLUMINATIR_MAX_COBSPACKET_SIZE (ILLUMINATIR_MAX_PACKET_SIZE + 1)

#define ILLUMINATIR_OFFSETARRAY_MIN_VALUES 1
#define ILLUMINATIR_OFFSETARRAY_MAX_VALUES 16


typedef enum {
	ILLUMINATIR_ERROR_NONE,
	ILLUMINATIR_ERROR_UNKNOWN,
	ILLUMINATIR_ERROR_BUFFER_OVERFLOW,
	ILLUMINATIR_ERROR_PACKET_TOO_SHORT,
	ILLUMINATIR_ERROR_PACKET_TOO_LONG,
	ILLUMINATIR_ERROR_UNSUPPORTED_VERSION,
	ILLUMINATIR_ERROR_UNSUPPORTED_FORMAT,
	ILLUMINATIR_ERROR_INVALID_SIZE,
	ILLUMINATIR_ERROR_INVALID_CRC,
	ILLUMINATIR_ERROR_NULL_BYTE,
	ILLUMINATIR_ERROR_NULL_POINTER,
} illuminatIr_error_t;

const char * illuminatIr_error_toProgString( illuminatIr_error_t err );

typedef void (*illuminatIr_parse_setChannel_t)( uint8_t channel, uint8_t value );
typedef void (*illuminatIr_parse_setConfig_t)( const char * key, uint8_t key_size, const uint8_t * values, uint8_t values_size );
illuminatIr_error_t illuminatIr_parse         ( const uint8_t * packet,     uint8_t packet_size,     illuminatIr_parse_setChannel_t setChannelFunc, illuminatIr_parse_setConfig_t setConfigFunc );
illuminatIr_error_t illuminatIr_parse_fromCobs( const uint8_t * cobsPacket, uint8_t cobsPacket_size, illuminatIr_parse_setChannel_t setChannelFunc, illuminatIr_parse_setConfig_t setConfigFunc );

illuminatIr_error_t IlluminatIr_build_offsetArray       ( uint8_t * packet,     uint8_t * packet_size,     uint8_t offset, const uint8_t * values, uint8_t values_size );
illuminatIr_error_t IlluminatIr_build_offsetArray_toCobs( uint8_t * cobsPacket, uint8_t * cobsPacket_size, uint8_t offset, const uint8_t * values, uint8_t values_size );
illuminatIr_error_t IlluminatIr_build_config            ( uint8_t * packet,     uint8_t * packet_size,     const char * key, uint8_t key_len, const uint8_t * values, uint8_t values_size );
illuminatIr_error_t IlluminatIr_build_config_toCobs     ( uint8_t * cobsPacket, uint8_t * cobsPacket_size, const char * key, uint8_t key_len, const uint8_t * values, uint8_t values_size );


#endif
