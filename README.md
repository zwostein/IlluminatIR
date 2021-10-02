# IlluminatIR

IlluminatIR synchronizes lighting devices (and possibly other gadgets) in real-time via a unidirectional infrared serial protocol.

This is the firmware for ATmega32U4 microcontrollers implementing the transmitter.  
The infrared transmitter is connected to a host computer via USB and acts as a HID device.
It is based on [LUFA](https://fourwalledcubicle.com/LUFA.php).


## How to build

### Obtaining sources:

	git clone https://github.com/abcminiuser/lufa.git
	git clone --recurse-submodules https://github.com/zwostein/IlluminatIR.git

### Compilation:

	cd IlluminatIR
	make

### Then to flash the software onto the microcontroller:

	make avrdude
