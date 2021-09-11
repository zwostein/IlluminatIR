/*
 * IlluminatIR
 * Copyright (C) 2021  zwostein
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "illuminatir.h"

#include "crc8.h"
#include "cobs.h"

#include <stdint.h>

#include <string.h>

#include <avr/pgmspace.h>


const char * illuminatIr_error_toProgString( illuminatIr_error_t err )
{
	switch( err )
	{
		case ILLUMINATIR_ERROR_NONE:                return PSTR("None");
		case ILLUMINATIR_ERROR_UNKNOWN:             return PSTR("Unknown");
		case ILLUMINATIR_ERROR_BUFFER_OVERFLOW:     return PSTR("Buffer overflow");
		case ILLUMINATIR_ERROR_PACKET_TOO_SHORT:    return PSTR("Packet too short to be valid");
		case ILLUMINATIR_ERROR_PACKET_TOO_LONG:     return PSTR("Packet too long");
		case ILLUMINATIR_ERROR_UNSUPPORTED_VERSION: return PSTR("Unsupported version");
		case ILLUMINATIR_ERROR_UNSUPPORTED_FORMAT:  return PSTR("Unsupported format");
		case ILLUMINATIR_ERROR_INVALID_SIZE:        return PSTR("Invalid size");
		case ILLUMINATIR_ERROR_INVALID_CRC:         return PSTR("Invalid CRC");
		case ILLUMINATIR_ERROR_NULL_BYTE:           return PSTR("Null Byte");
		case ILLUMINATIR_ERROR_NULL_POINTER:        return PSTR("Null Pointer");
	}
	return NULL;
}


illuminatIr_error_t illuminatIr_parse( const uint8_t * packet, uint8_t packet_size, illuminatIr_parse_setChannel_t setChannelFunc, illuminatIr_parse_setConfig_t setConfigFunc )
{
	if( !packet ) {
		return ILLUMINATIR_ERROR_NULL_POINTER;
	}
	if( packet_size < ILLUMINATIR_MIN_PACKET_SIZE ) {
		return ILLUMINATIR_ERROR_PACKET_TOO_SHORT;
	}
	uint8_t version = packet[0] >> 6;
	if( version != 0 ) {
		return ILLUMINATIR_ERROR_UNSUPPORTED_VERSION;
	}
	uint8_t format = (packet[0] & 0b00110000) >> 4;
	uint8_t payload_size = (packet[0] & 0b00001111) + 2;
	if( 1 + payload_size + 1 != packet_size ) {
		return ILLUMINATIR_ERROR_INVALID_SIZE;
	}
	const uint8_t * payload = packet + 1;
	uint8_t crc_received = packet[packet_size - 1];
	uint8_t crc_calculated = crc8( packet, packet_size - 1, CRC8_INITIAL_SEED );
	if( crc_received != crc_calculated ) {
		return ILLUMINATIR_ERROR_INVALID_CRC;
	}
	
	switch( format ) {
		case 0: { // Offset + Values
			if( !setChannelFunc ) {
				return ILLUMINATIR_ERROR_NONE;
			}
			uint8_t offset = *payload++;
			for( uint8_t i = 0; i < payload_size - 1; i++ ) {
				uint8_t channel = i+offset;
				setChannelFunc( channel, *payload++ );
			}
			return ILLUMINATIR_ERROR_NONE;
		}
		case 1: { // Channel:Value pairs
			if( !setChannelFunc ) {
				return ILLUMINATIR_ERROR_NONE;
			}
			uint8_t pairs = payload_size / 2;
			uint8_t halfpair = payload_size % 2;
			while( pairs ) {
				uint8_t channel = *payload++;
				uint8_t value = *payload++;
				setChannelFunc( channel, value );
				pairs--;
			}
			if( halfpair ) {
				uint8_t channel = *payload++;
				setChannelFunc( channel, 0 );
			}
			return ILLUMINATIR_ERROR_NONE;
		}
		case 2: { // Configuration Key:Value(s) pair
			if( !setConfigFunc ) {
				return ILLUMINATIR_ERROR_NONE;
			}
			const char * key = (const char *)payload;
			uint8_t key_size = strnlen( key, payload_size );
			uint8_t values_size = payload_size - key_size;
			if( values_size > 0 ) {
				values_size--;
			}
			const uint8_t * values = payload + payload_size - values_size;
			setConfigFunc( key, key_size, values, values_size );
			return ILLUMINATIR_ERROR_NONE;
		}
		default: {
			return ILLUMINATIR_ERROR_UNSUPPORTED_FORMAT;
		}
	}
	return ILLUMINATIR_ERROR_UNKNOWN;
}

illuminatIr_error_t illuminatIr_parse_fromCobs( const uint8_t * cobsPacket, uint8_t cobsPacket_size, illuminatIr_parse_setChannel_t setChannelFunc, illuminatIr_parse_setConfig_t setConfigFunc )
{
	uint8_t packet[ILLUMINATIR_MAX_PACKET_SIZE + 1];
	cobs_decode_result res = cobs_decode( packet, sizeof(packet), cobsPacket, cobsPacket_size );
	switch( res.status ) {
		case COBS_DECODE_INPUT_TOO_SHORT:     return ILLUMINATIR_ERROR_PACKET_TOO_SHORT;
		case COBS_DECODE_ZERO_BYTE_IN_INPUT:  return ILLUMINATIR_ERROR_NULL_BYTE;
		case COBS_DECODE_OUT_BUFFER_OVERFLOW: return ILLUMINATIR_ERROR_PACKET_TOO_LONG;
		case COBS_DECODE_NULL_POINTER:        return ILLUMINATIR_ERROR_NULL_POINTER;
		case COBS_DECODE_OK:                  return illuminatIr_parse( packet, res.out_len, setChannelFunc, setConfigFunc );
	}
	return ILLUMINATIR_ERROR_UNKNOWN;
}


illuminatIr_error_t IlluminatIr_build_offsetArray( uint8_t * packet, uint8_t * packet_size, uint8_t offset, const uint8_t * values, uint8_t values_size )
{
	if( !packet_size ) {
		return ILLUMINATIR_ERROR_NULL_POINTER;
	}
	if( values_size < ILLUMINATIR_OFFSETARRAY_MIN_VALUES ) {
		return ILLUMINATIR_ERROR_INVALID_SIZE;
	}
	if( values_size > ILLUMINATIR_OFFSETARRAY_MAX_VALUES ) {
		return ILLUMINATIR_ERROR_INVALID_SIZE;
	}
	uint8_t packet_size_available = *packet_size;
	*packet_size = 1 + 1 + values_size + 1;
	if( packet_size_available < *packet_size ) {
		return ILLUMINATIR_ERROR_BUFFER_OVERFLOW;
	}
	uint8_t * p = packet;
	*p++ = 0
	     | (values_size - 1)
	     ;
	*p++ = offset;
	while( values_size-- ) {
		*p++ = *values++;
	}
	*p++ = crc8( packet, (*packet_size) - 1, CRC8_INITIAL_SEED );
	return ILLUMINATIR_ERROR_NONE;
}

illuminatIr_error_t IlluminatIr_build_config( uint8_t * packet, uint8_t * packet_size, const char * key, uint8_t key_len, const uint8_t * values, uint8_t values_size )
{
	if( !packet_size ) {
		return ILLUMINATIR_ERROR_NULL_POINTER;
	}
	uint8_t payload_size = key_len + 1 + values_size;
	if( payload_size < 2 ) {
		return ILLUMINATIR_ERROR_INVALID_SIZE;
	}
	if( payload_size > 15 + 2 ) {
		return ILLUMINATIR_ERROR_INVALID_SIZE;
	}
	uint8_t packet_size_available = *packet_size;
	*packet_size = 1 + payload_size + 1;
	if( packet_size_available < *packet_size ) {
		return ILLUMINATIR_ERROR_BUFFER_OVERFLOW;
	}
	uint8_t * p = packet;
	*p++ = 0
	     | (0b10 << 4)
	     | (payload_size - 2)
	     ;
	memcpy( p, key, key_len );
	p += key_len;
	*p++ = 0;
	memcpy( p, values, values_size );
	p += values_size;
	*p++ = crc8( packet, (*packet_size)-1, CRC8_INITIAL_SEED );
	return ILLUMINATIR_ERROR_NONE;
}

illuminatIr_error_t IlluminatIr_toCobs( uint8_t * cobsPacket, uint8_t * cobsPacket_size, const uint8_t * packet, uint8_t packet_size )
{
	if( !cobsPacket_size ) {
		return ILLUMINATIR_ERROR_NULL_POINTER;
	}
	cobs_encode_result res = cobs_encode( cobsPacket, *cobsPacket_size, packet, packet_size );
	switch( res.status ) {
		case COBS_ENCODE_NULL_POINTER:        return ILLUMINATIR_ERROR_NULL_POINTER;
		case COBS_ENCODE_OUT_BUFFER_OVERFLOW: return ILLUMINATIR_ERROR_BUFFER_OVERFLOW;
		case COBS_ENCODE_OK:
			*cobsPacket_size = res.out_len;
			return ILLUMINATIR_ERROR_NONE;
	}
	return ILLUMINATIR_ERROR_UNKNOWN;
}

illuminatIr_error_t IlluminatIr_build_offsetArray_toCobs( uint8_t * cobsPacket, uint8_t * cobsPacket_size, uint8_t offset, const uint8_t * values, uint8_t values_size )
{
	uint8_t packet[ILLUMINATIR_MAX_PACKET_SIZE] = { 0 };
	uint8_t packet_size = sizeof(packet);
	illuminatIr_error_t err = IlluminatIr_build_offsetArray( packet, &packet_size, offset, values, values_size );
	if( err != ILLUMINATIR_ERROR_NONE ) {
		return err;
	}
	return IlluminatIr_toCobs( cobsPacket, cobsPacket_size, packet, packet_size );
}

illuminatIr_error_t IlluminatIr_build_config_toCobs( uint8_t * cobsPacket, uint8_t * cobsPacket_size, const char * key, uint8_t key_len, const uint8_t * values, uint8_t values_size )
{
	uint8_t packet[ILLUMINATIR_MAX_PACKET_SIZE] = { 0 };
	uint8_t packet_size = sizeof(packet);
	illuminatIr_error_t err = IlluminatIr_build_config( packet, &packet_size, key, key_len, values, values_size );
	if( err != ILLUMINATIR_ERROR_NONE ) {
		return err;
	}
	return IlluminatIr_toCobs( cobsPacket, cobsPacket_size, packet, packet_size );
}
