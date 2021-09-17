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

#include "config.h"
#include "Descriptors.h"
#include "crc8.h"
#include "illuminatir.h"
#include "cobs.h"
#include "avr-uart/uart.h"

#include <inttypes.h>
#include <math.h>
#include <assert.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>


/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
//static uint8_t PrevHIDReportBuffer[ILLUMINATIR_MAX_CHANNELS];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Generic_HID_Interface =
{
	.Config = {
		.InterfaceNumber        = INTERFACE_ID_GenericHID,
		.ReportINEndpoint       =
			{
				.Address        = GENERIC_IN_EPADDR,
				.Size           = GENERIC_EPSIZE,
				.Banks          = 1,
			},
		.PrevReportINBuffer     = NULL/*PrevHIDReportBuffer*/,
		.PrevReportINBufferSize = 0/*sizeof(PrevHIDReportBuffer)*/,
	},
};


/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
	.Config = {
		.ControlInterfaceNumber = INTERFACE_ID_CDC_CCI,
		.DataINEndpoint         =
			{
				.Address        = CDC_TX_EPADDR,
				.Size           = CDC_TXRX_EPSIZE,
				.Banks          = 1,
			},
		.DataOUTEndpoint        =
			{
				.Address        = CDC_RX_EPADDR,
				.Size           = CDC_TXRX_EPSIZE,
				.Banks          = 1,
			},
		.NotificationEndpoint   =
			{
				.Address        = CDC_NOTIFICATION_EPADDR,
				.Size           = CDC_NOTIFICATION_EPSIZE,
				.Banks          = 1,
			},
	},
};


static volatile uint16_t iterationCounter = 0;
static volatile bool usb_ready = false;

/*
int wrap_uart_putchar( char c, FILE * stream )
{
	uart1_putc(c);
	return 0;
}
static FILE uart = FDEV_SETUP_STREAM( wrap_uart_putchar, NULL, _FDEV_SETUP_WRITE );
*/
int wrap_usb_putchar( char c, FILE * stream )
{
	(void) stream;
	if( !usb_ready ) {
		return -1;
	}
	switch( CDC_Device_SendByte( &VirtualSerial_CDC_Interface, c ) ) {
		case ENDPOINT_READYWAIT_NoError:
			return 0;
		default:
			return -1;
	}
}
int wrap_usb_getchar( FILE * stream )
{
	(void) stream;
	if( !usb_ready ) {
		return -1;
	}
	return CDC_Device_ReceiveByte( &VirtualSerial_CDC_Interface );
}
static FILE usb = FDEV_SETUP_STREAM( wrap_usb_putchar, wrap_usb_getchar, _FDEV_SETUP_RW );


#define MODULATOR_CLK_DIV 1

bool modulator_set( uint32_t freq )
{
	if( !freq ) {
		return false;
	}
	uint32_t raw = F_CPU / ( (uint32_t)freq * MODULATOR_CLK_DIV * 2 );
	// also have to subtract one - but prevent wraparound
	if( raw ) {
		raw--;
	}
	if( raw > UINT16_MAX ) {
		return false;
	}
	OCR1A = raw;
//	OCR1B = raw;
	return true;
}

void modulator_init( uint32_t initial_freq )
{
	modulator_set( initial_freq );
	TCCR1C = 0
	       ;
	TCCR1B = 0
	       | _BV(WGM12) // Clear Timer on Compare Match (CTC). TOP = OCR1A
#if MODULATOR_CLK_DIV == 1
	       | _BV(CS10) // clk/1.
#elif MODULATOR_CLK_DIV == 8
	       | _BV(CS11) // clk/8.
#elif MODULATOR_CLK_DIV == 64
	       | _BV(CS10) // clk/64.
	       | _BV(CS11) // clk/64.
#elif MODULATOR_CLK_DIV == 256
	       | _BV(CS12) // clk/256.
#elif MODULATOR_CLK_DIV == 1024
	       | _BV(CS10) // clk/1024.
	       | _BV(CS12) // clk/1024.
#else
#	error "Unsupported clock divisor for modulator!"
#endif
	       ;
	TCCR1A = 0
	       | _BV(COM1A0) // Toggle OC1A (PB5) on compare match.
	       | _BV(COM1B0) // Toggle OC1B (PB6) on compare match.
	       ;
	DDRB |= _BV(PB5);  // Modulator pin as output
	DDRB |= _BV(PB6);  // Modulator pin as output
}


void SetupHardware( void )
{
	MCUSR = 0;
	wdt_enable( WDTO_2S );

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
	modulator_init( INITIAL_MODULATION );
	uart1_init( UART_BAUD_SELECT(INITIAL_UART_BAUDRATE, F_CPU) );
}


#define PACKET_TX_ISEMPTY() (uart1_tx_available() >= UART_TX1_BUFFER_SIZE-1)
#define PACKET_TX_AVAILABLE() uart1_tx_available()
#define PACKET_TX(x) uart1_putc(x)

int8_t sendPacket( const uint8_t * ptr, uint8_t size )
{
	fprintf_P( &usb, PSTR("Sending: ") );
	for( uint8_t i = 0; i < size; i++ ) {
		fprintf( &usb, "%02x", (int)ptr[i] );
	}
	fprintf( &usb, " ... " );

	if( size > PACKET_TX_AVAILABLE() ) {
		fprintf_P( &usb, PSTR("Failed!\n") );
		return -1;
	}
	for( uint8_t i = 0; i < size; i++ ) {
		PACKET_TX( ptr[i] );
	}
	fprintf_P( &usb, PSTR("Ok.\n") );
	return 0;
}

void sendSync( void )
{
	PACKET_TX( 0 );
}

typedef enum {
	READHEX_ERROR_INVALID_CHAR = -1,
	READHEX_ERROR_ODD_LENGTH = -2,
	READHEX_ERROR_DESTINATION_TOO_SMALL = -3,
} readHex_error_t;

int readHex_digit( char c )
{
	if( c >= '0' && c <= '9' ) {
		return c - '0';
	} else if( c >= 'a' && c <= 'f' ) {
		return c - 'a' + 0xa;
	} else if( c >= 'A' && c <= 'F' ) {
		return c - 'A' + 0xa;
	} else {
		return READHEX_ERROR_INVALID_CHAR;
	}
}

int readHex_string( uint8_t * dst, size_t dst_size, const char * str, size_t str_len )
{
	if( str_len & 1 ) {
		return READHEX_ERROR_ODD_LENGTH;
	}
	uint8_t size = str_len / 2;
	if( size > dst_size ) {
		return READHEX_ERROR_DESTINATION_TOO_SMALL;
	}
	for( uint8_t i = 0; i < str_len; i++ ) {
		uint8_t j = i/2;
		int ret = readHex_digit( str[i] );
		if( ret < 0 ) {
			return ret;
		}
		dst[j] = ret << 4;
		i++;
		ret = readHex_digit( str[i] );
		if( ret < 0 ) {
			return ret;
		}
		dst[j] |= ret & 0x00ff;
	}
	return size;
}


void print_readHex_error( int err )
{
	switch( err ) {
		case READHEX_ERROR_ODD_LENGTH:            fprintf_P( &usb, PSTR("Odd number of hex digits!\n") );      break;
		case READHEX_ERROR_DESTINATION_TOO_SMALL: fprintf_P( &usb, PSTR("Too much hex data!\n") );             break;
		case READHEX_ERROR_INVALID_CHAR:          fprintf_P( &usb, PSTR("Invalid character in hex data!\n") ); break;
	}
}


void wdtTrigger( void )
{
	unsigned i = 0;
	while( true ) {
		_delay_ms(1.0);
		if( !(i % 100) ) {
			printf_P( PSTR(".\n") );
		}
		USB_USBTask(); // too keep USB serial output alive until watchdog triggers
		i++;
	}
}


void parseLine( char * line )
{
	char * token = strtok( line, " \t" );
	if( !token ) { fprintf_P( &usb, PSTR("No command given!\n") ); return; }
	if( !strcasecmp_P( token, PSTR("set") ) ) {
		token = strtok( NULL, "=: \t" );
		if( !token ) { fprintf_P( &usb, PSTR("No variable given!\n") ); return; }
		if( !strcasecmp_P( token, PSTR("modulation_hz") ) ) {
			token = strtok( NULL, "" );
			if( !token ) { fprintf_P( &usb, PSTR("No value given!\n") ); return; }
			uint32_t freq = INITIAL_MODULATION;
			if( !sscanf( token, "%"PRIu32, &freq ) )
			{
				fprintf_P( &usb, PSTR("Failed to parse \"%s\" as a frequency in Hz!\n"), token );
				return;
			}
			if( modulator_set(freq) ) {
				fprintf_P( &usb, PSTR("Modulating @ %"PRIu32"Hz\n"), freq );
			} else {
				fprintf_P( &usb, PSTR("Failed to set modulator to %"PRIu32"Hz!\n"), freq );
				return;
			}
		} else {
			fprintf_P( &usb, PSTR("Unknown variable \"%s\"!\n"), token );
			return;
		}
	} else if( !strcasecmp_P( token, PSTR("sendconfig") ) ) {
		char * key = strtok( NULL, " \t" );
		if( !key ) { fprintf_P( &usb, PSTR("No config key given!\n") ); return; }
		char * values_hex = strtok( NULL, "" );
		uint8_t values[16];
		int values_size = readHex_string( values, sizeof(values), values_hex, strlen(values_hex) );
		if( values_size < 0 ) { print_readHex_error( values_size ); return; }
		uint8_t cobsPacket[ILLUMINATIR_MAX_COBSPACKET_SIZE];
		uint8_t cobsPacket_size = sizeof(cobsPacket);
		illuminatIr_error_t err = IlluminatIr_build_config_toCobs( cobsPacket, &cobsPacket_size, key, strlen(key), values, values_size );
		if( err != ILLUMINATIR_ERROR_NONE ) {
			fprintf_P( &usb, PSTR("Could not build packet: %S\n"), illuminatIr_error_toProgString(err) );
			return;
		}
		if( sendPacket( cobsPacket, cobsPacket_size ) < 0 ) {
			fprintf_P( &usb, PSTR("Could not send packet!\n") );
			return;
		}
		sendSync();
	} else if( !strcasecmp_P( token, PSTR("sendraw") ) ) {
		token = strtok( NULL, "" );
		if( !token ) { fprintf_P( &usb, PSTR("No raw hex data given!\n") ); return; }
		uint8_t packet[32];
		int packet_size = readHex_string( packet, sizeof(packet), token, strlen( token ) );
		if( packet_size < 0 ) { print_readHex_error( packet_size ); return; }
		sendPacket( packet, packet_size );
	} else if( !strcasecmp_P( token, PSTR("bootloader") ) ) {
		printf_P( PSTR("Rebooting into Bootloader!\n") );
		//NOTE: This is specific to the Arduino Caterina Bootloader! But other boards like the Pololu A-Star series might come with a compatible bootloader.
		const uint16_t bootKey = 0x7777;
		volatile uint16_t *const bootKeyPtr = (volatile uint16_t *)0x0800;
		*bootKeyPtr = bootKey;
		wdtTrigger();
	} else {
		fprintf_P( &usb, PSTR("Unknown command \"%s\"!\n"), token );
		return;
	}
}


static uint8_t ledValues_prev[ILLUMINATIR_MAX_CHANNELS] = { 0 };
static uint8_t ledValues[ILLUMINATIR_MAX_CHANNELS] = { 0 };
static size_t  ledValues_changed = 0; // index of the last index that changed + 1

static int getNextDiff( const uint8_t * a, const uint8_t * b, size_t from, size_t to )
{
	for( ; from < to; from++ ) {
		if( a[from] != b[from] ) {
			return from;
		}
	}
	return -1;
}

static int getNextGap( const uint8_t * a, const uint8_t * b, size_t from, size_t to, size_t min )
{
	if( min == 0 ) {
		min = 1;
	}
	size_t min_left = min;
	for( ; from < to; from++ ) {
		if( a[from] == b[from] ) {
			min_left--;
			if( min_left == 0 ) {
				return from - min + 1;
			}
		} else {
			min_left = min;
		}
	}
	return -1;
}

static bool updateLedValues( void )
{
	static size_t updateStart = 0;
	size_t checked = 0;
// 	fprintf_P( &usb, PSTR("updateLedValues:\n") );
	while( checked < ILLUMINATIR_MAX_CHANNELS ) {
//  		fprintf_P( &usb, PSTR("\tupdateStart: %u\n"), updateStart );
		int nextDiff = getNextDiff( ledValues, ledValues_prev, updateStart, ILLUMINATIR_MAX_CHANNELS );
//  		fprintf_P( &usb, PSTR("\tnextDiff: %d\n"), nextDiff );
		if( nextDiff < 0 ) {
			checked += ILLUMINATIR_MAX_CHANNELS-updateStart;
			nextDiff = getNextDiff( ledValues, ledValues_prev, 0, updateStart );
//  			fprintf_P( &usb, PSTR("\tnextDiff2: %d\n"), nextDiff );
			if( nextDiff < 0 )
				return true; // no differences
			checked += nextDiff;
		}
		size_t blockStart = nextDiff;

		int nextGap = getNextGap( ledValues, ledValues_prev, blockStart, ILLUMINATIR_MAX_CHANNELS, 3 );
//  		fprintf_P( &usb, PSTR("\tgetNextGap: %d\n"), nextGap );
		size_t blockEnd = 0;
		if( nextGap < 0 ) {
			blockEnd = ILLUMINATIR_MAX_CHANNELS;
		} else if ( nextGap-blockStart > ILLUMINATIR_OFFSETARRAY_MAX_VALUES ) {
			blockEnd = ILLUMINATIR_OFFSETARRAY_MAX_VALUES;
		} else {
			blockEnd = nextGap;
		}

		size_t blockLen = blockEnd-blockStart;

//  		fprintf_P( &usb, PSTR("\tblockStart: %u, blockEnd:%u\n"), blockStart, blockEnd );
		uint8_t cobsPacket[ILLUMINATIR_MAX_COBSPACKET_SIZE];
		uint8_t cobsPacket_size = sizeof(cobsPacket);
		illuminatIr_error_t err = IlluminatIr_build_offsetArray_toCobs( cobsPacket, &cobsPacket_size, blockStart, &ledValues[blockStart], blockLen );
		if( err != ILLUMINATIR_ERROR_NONE ) {
			fprintf_P( &usb, PSTR("Could not build packet: %S\n"), illuminatIr_error_toProgString(err) );
			return false;
		}
		if( sendPacket( cobsPacket, cobsPacket_size ) < 0 ) {
			return false; // packet not sent
		}
		sendSync();

		memcpy( &ledValues_prev[blockStart], &ledValues[blockStart], blockLen );

		updateStart = blockEnd;
		if( updateStart >= ILLUMINATIR_MAX_CHANNELS ) {
			updateStart -= ILLUMINATIR_MAX_CHANNELS;
		}
		checked += blockLen;
// 		fprintf_P( &usb, PSTR("\tchecked: %u\n"), checked );
	}
	return true; // iterated once over every LED value
}

#define LEDVALUE_SCRUBBING_SIZE 8

int main( void )
{
	SetupHardware();
	GlobalInterruptEnable();

	LEDs_SetAllLEDs( LEDS_NO_LEDS );
	fprintf_P( &usb, PSTR("IlluminatIR powered on.\n") );

	char line[256] = {0};
	uint8_t line_pos = 0;
	bool line_ignore = false;

	uint8_t ledValues_scrubbingPos = 0;

	while( true ) {
		if( !iterationCounter ) {
			fprintf_P( &usb, PSTR("IlluminatIR\n") );
		}

		if( !ledValues_changed && PACKET_TX_ISEMPTY() ) {
			for( uint8_t i = 0; i < LEDVALUE_SCRUBBING_SIZE; i++ ) {
				ledValues_prev[ledValues_scrubbingPos]++;
				ledValues_scrubbingPos++;
				if( ledValues_scrubbingPos >= ILLUMINATIR_MAX_CHANNELS ) {
					ledValues_scrubbingPos=0;
					break;
				}
			}
			ledValues_changed = LEDVALUE_SCRUBBING_SIZE;
		}
		if( ledValues_changed ) {
			if( updateLedValues() ) {
				ledValues_changed = 0;
			}
		}

		// HID
		HID_Device_USBTask( &Generic_HID_Interface );

		// Serial
		int16_t rec = fgetc( &usb ); // Must throw away unused bytes from USB host, or it will lock up while waiting for the device
		if( rec >= 0 && rec != '\r' ) {
			if( rec == '\n' ) {
				if( !line_ignore ) {
					assert( line_pos < sizeof(line)-1 );
					line[line_pos] = 0;
					fprintf_P( &usb, PSTR("Input line: \"%s\"\n"), line );
					parseLine( line );
				}
				line_pos = 0;
				line_ignore = false;
			} else {
				// must be space left for received character + null byte
				if( line_pos >= sizeof(line)-2 ) {
					fprintf_P( &usb, PSTR("Input line overflow!\n") );
					line_ignore = true;
				} else {
					line[line_pos] = rec;
					line_pos++;
				}
			}
		}
		CDC_Device_USBTask( &VirtualSerial_CDC_Interface );

		// finish iteration
		USB_USBTask();
		wdt_reset();
		iterationCounter++;
	}
}


/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect( void )
{
	//LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}


/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect( void )
{
	//LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}


/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged( void )
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Generic_HID_Interface);
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	USB_Device_EnableSOFEvents();

	//LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}


/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest( void )
{
	HID_Device_ProcessControlRequest( &Generic_HID_Interface);
	CDC_Device_ProcessControlRequest( &VirtualSerial_CDC_Interface );
}


/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame( void )
{
	HID_Device_MillisecondElapsed( &Generic_HID_Interface );
}


/** CDC class driver callback function the processing of changes to the virtual
 *  control lines sent from the host..
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged( USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo )
{
	/* You can get changes to the virtual CDC lines in this callback; a common
	   use-case is to use the Data Terminal Ready (DTR) flag to enable and
	   disable CDC communications in your application when set to avoid the
	   application blocking while waiting for a host to become ready and read
	   in the pending data from the USB endpoints.
	*/
	usb_ready = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR) != 0;
}


static const char * CDC_LineEncodingFormat_toString( uint8_t f )
{
	switch(f) {
		case CDC_LINEENCODING_OneStopBit:          return "1";
		case CDC_LINEENCODING_OneAndAHalfStopBits: return "1.5";
		case CDC_LINEENCODING_TwoStopBits:         return "2";
		default:                                   return "?";
	}
}


static const char * CDC_LineEncodingParity_toString (uint8_t p )
{
	switch(p) {
		case CDC_PARITY_None:  return "None";
		case CDC_PARITY_Odd:   return "Odd";
		case CDC_PARITY_Even:  return "Even";
		case CDC_PARITY_Mark:  return "Mark";
		case CDC_PARITY_Space: return "Space";
		default:               return "?";
	}
}


void EVENT_CDC_Device_LineEncodingChanged( USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo )
{
	fprintf_P( &usb, PSTR("Line encoding changed: %"PRIu32" Baud, %s Stopbits, %d Databits, %s Parity\n"),
	           CDCInterfaceInfo->State.LineEncoding.BaudRateBPS,
	           CDC_LineEncodingFormat_toString(CDCInterfaceInfo->State.LineEncoding.CharFormat),
	           (unsigned)CDCInterfaceInfo->State.LineEncoding.DataBits,
	           CDC_LineEncodingParity_toString(CDCInterfaceInfo->State.LineEncoding.ParityType) );
}


/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport( USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          uint8_t* const ReportID,
                                          const uint8_t ReportType,
                                          void* ReportData,
                                          uint16_t* const ReportSize )
{
	(void) HIDInterfaceInfo;
	(void) ReportID;
	(void) ReportType;
	(void) ReportData;
	//uint8_t * ReportData = (uint8_t*)_ReportData;
	*ReportSize = 0;
	return false;
}


/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport( USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                           const uint8_t ReportID,
                                           const uint8_t ReportType,
                                           const void* ReportData,
                                           const uint16_t ReportSize )
{
	(void) HIDInterfaceInfo;
	(void) ReportType;
	//const uint8_t * ReportData = (uint8_t*)_ReportData;
	switch( ReportID ) {
		case 0x01: {
			ledValues_changed = MIN( ReportSize, ILLUMINATIR_MAX_CHANNELS );
			memcpy( ledValues, ReportData, ledValues_changed );
			break;
		}
	}
}
