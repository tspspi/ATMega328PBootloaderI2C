#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/twi.h>

/*
	I2C bootloader for ATMega328P microcontroller using Arduino compatible
	protocol via I2C. This allows flashing the microcontrollers application
	via the I2C bus instead of a serial port or the SPI interface.
*/

// Maximum number of errors before we leave the bootloader and enter the application
#ifndef MAX_ERROR_COUNT
	#define MAX_ERROR_COUNT 10
#endif

#ifndef HW_VERSION
	#define HW_VERSION 0x02
#endif
#ifndef SW_VERSION_MAJOR
	#define SW_VERSION_MAJOR 0x01
#endif
#ifndef SW_VERSION_MINOR
	#define SW_VERSION_MINOR 0x10
#endif

#ifndef I2CADR
	#define I2CADR 0x14
	#warning "Using default I2C address 0x14 because none has been specified"
#endif

#define SIGNATUREBYTE_1		0x1E /*ATMEL*/
#if defined(__AVR_ATmega328P__)
	#define SIGNATUREBYTE_2	0x95
	#define SIGNATUREBYTE_3	0x0F
	#define PAGESIZE		0x40U
#else
	#error "Invalid or unknown platform"
#endif

// #define I2CGENERALCALL

/*
	The receive timeout specifies the maximum time the bootloader is
	waiting for incoming characters.

	Using F_CPU >> 4 means for 16 MHz CPU 62.5 ms * cycles in read loop.
*/
#ifndef RECEIVE_TIMEOUT
	#define RECEIVE_TIMEOUT (F_CPU >> 4)
#endif

/*
	Basic bootloader flow:

		- Check if we have been reset by an external reset or not. If not
		  (i.e. if the rest has been triggered by brownout, power-on reset
	  	  or the watchdog) we directly enter the application. Else we
		  enter the bootloader.
		- In case we entered the bootloader we initialize the TWI interface
		  with the programmed address (also listening for general calls).
		  Supporting general call allows bulk-programming a whole bunch
		  of boards at once - this should only be done if the I2C address
		  is not hardcoded inside the binary but somehow determined externally
		  or via EEPROM.
		- Then we enter an infinite loop to wait for commands from the
		  programming hardware. In case we dont receive any commands during
		  a specific timeout we will disable the TWI interface again, leave
		  the bootloader and enter the application
*/

#define CMD__STK_GET_SYNC					0x30
#define CMD__STK_GET_SIGN_ON				0x31
#define CMD__STK_SET_PARAMETER				0x40
#define CMD__STK_GET_PARAMETER				0x41
#define CMD__STK_SET_DEVICE					0x42
#define CMD__STK_SET_DEVICE_EXT				0x45
#define CMD__STK_ENTER_PROGRAMMING_MODE		0x50
#define CMD__STK_LEAVE_PROGRAMMING_MODE		0x51
#define CMD__STK_ERASE_DEVICE				0x52
#define CMD__STK_LOAD_ADDRESS				0x55
#define CMD__STK_UNIVERSAL					0x56
#define CMD__STK_PROGPAGE					0x64
#define CMD__STK_READPAGE					0x74
#define CMD__STK_READ_SIGNATURE				0x75
#define CMD__STK_READ_OSCCAL				0x76

#define STKPARAM__HARDWARE_VERSION			0x80
#define STKPARAM__SOFTWARE_VERSION_MAJOR	0x81
#define STKPARAM__SOFTWARE_VERSION_MINOR	0x82
#define STKPARAM__SCK_DURATION				0x98

/*
	Loader state
*/
static uint8_t errorCounter;
static uint16_t address;
static uint8_t buffer[256]; /* Used to buffer a whole page before executing programming instruction */


static void reset() {
	/* Reset via watchdog after 16 ms delay */
	WDTCSR = 0x80; /* Enable watchdog with 16 ms delay */
	for(;;) {
		/* Busy wait till reset ... */
	}
}

/*
	Pointer to start of application - this is used to call into the
	application.
*/
void (*applicationMain)(void) = 0x0000;

static inline void enterApplication() {
	/* Disable TWI */
	TWCR = 0x80; /* Clear interrupt bit if set and disable TWI interface */

	/* Enter main in an endless loop (in case it will ever return) */
	for(;;) {
		applicationMain();
	}
}

static uint8_t i2cReceiveNextByte() {
	uint32_t timeoutCounter = 0;

	for(;;) {
		while((TWCR & 0x80) == 0) {
			timeoutCounter = timeoutCounter + 1;
			if(timeoutCounter >= RECEIVE_TIMEOUT) {
				/* We havent received anything inside our listening window */
				applicationMain();
			}
		}

		if(TW_STATUS == TW_SR_DATA_ACK) {
			/* We have received a byte */
			break;
		}

		/* In any other case we wait for the next event but do NOT reset our timeout */
	}

	uint8_t receivedData = TWDR;
	TWCR = 0xC4; /* Reset interrupt bit and wait for next cycle */
	return receivedData;
}

static void i2cTransmitNextByte(uint8_t dataByte) {
	uint32_t timeoutCounter = 0;

	for(;;) {
		while((TWCR & 0x80) == 0) {
			timeoutCounter = timeoutCounter + 1;
			if(timeoutCounter >= RECEIVE_TIMEOUT) {
				/* We havent received anything inside our listening window */
				applicationMain();
			}
		}

		if(TW_STATUS == TW_ST_DATA_ACK) {
			/* We have received a read request so we can transmit our data byte */
			break;
		}
	}
	TWDR = dataByte;
	TWCR = 0xC4; /* Reset interrupt bit and wait for next cycle */
	return;
}

static void responseEmpty() {
	if(i2cReceiveNextByte() == 0x20) {
		i2cTransmitNextByte(0x14);
		i2cTransmitNextByte(0x10);
	} else {
		if((errorCounter = errorCounter + 1) > MAX_ERROR_COUNT) {
			/*
				In case we have received more errornous messages than
				MAX_ERROR_COUNT we simply leave the bootloader and
				boot the application as usual.
			*/
			enterApplication();
		}
	}
}
static void responseByteSingle(uint8_t resp) {
	if(i2cReceiveNextByte() == 0x20) {
		i2cTransmitNextByte(0x14);
		i2cTransmitNextByte(resp);
		i2cTransmitNextByte(0x10);
	} else {
		if((errorCounter = errorCounter + 1) > MAX_ERROR_COUNT) {
			/*
				In case we have received more errornous messages than
				MAX_ERROR_COUNT we simply leave the bootloader and
				boot the application as usual.
			*/
			enterApplication();
		}
	}
}

static uint16_t length;

int main(void) {
	/*
		Check if we have been reset by an external reset. If not enter
		application (with WDT disabled)
	*/
	uint8_t mcuStatusRegister = MCUSR;
	WDTCSR = 0x18; /* Enable write access to the WDE for the next 4 cycles by setting WDE and WDCE */
	WDTCSR = 0x00; /* Disable watchdog */

	if((mcuStatusRegister & 0x0D) != 0) {
		/*
			Reset has been triggered by anything else than external reset.
			We start the application as usual
		*/
		enterApplication();
	}

	/*
		Reset error counter
	*/
	errorCounter = 0;

	/*
		Initialize the TWI interface
	*/
	cli();
	#ifndef I2CGENERALCALL
		TWAR = (I2CADR << 1) | 0x00;
	#else
		TWAR = (I2CADR << 1) | 0x01;
	#endif
	TWCR = 0xC4;
	sei();

	for(;;) {
		uint8_t cmdChar = i2cReceiveNextByte();

		if(cmdChar == CMD__STK_GET_SYNC) {
			responseEmpty();
		} else if(cmdChar == CMD__STK_GET_SIGN_ON) {
			/*
				Programmer ID has been requested. Respond if command
				has been correctly terminated
			*/
			if(i2cReceiveNextByte() == 0x20) {
				i2cTransmitNextByte(0x14);
				i2cTransmitNextByte('A');
				i2cTransmitNextByte('V');
				i2cTransmitNextByte('R');
				i2cTransmitNextByte(' ');
				i2cTransmitNextByte('I');
				i2cTransmitNextByte('S');
				i2cTransmitNextByte('P');
				i2cTransmitNextByte(0x10);
			} else {
				if((errorCounter = errorCounter + 1) > MAX_ERROR_COUNT) {
					/*
						In case we have received more errornous messages than
						MAX_ERROR_COUNT we simply leave the bootloader and
						boot the application as usual.
					*/
					enterApplication();
				}
			}
		} else if(cmdChar == CMD__STK_SET_PARAMETER) {
			/*
				This operation is simply ignored (from STK500)
			*/
			uint8_t cmdChar2 = i2cReceiveNextByte();
			if(cmdChar2 > 0x85) {
				i2cReceiveNextByte();
			}
			responseEmpty();
		} else if(cmdChar == CMD__STK_GET_PARAMETER) {
			/* Get device parameter. Currently we support hwardware version, software version and (required by AVR Studio) SCK Duration */
			uint8_t parameterType = i2cReceiveNextByte();
			if(parameterType == STKPARAM__HARDWARE_VERSION) 			{ responseByteSingle(HW_VERSION);		}
			else if(parameterType == STKPARAM__SOFTWARE_VERSION_MAJOR)	{ responseByteSingle(SW_VERSION_MAJOR);	}
			else if(parameterType == STKPARAM__SOFTWARE_VERSION_MINOR)	{ responseByteSingle(SW_VERSION_MINOR);	}
			else if(parameterType == STKPARAM__SCK_DURATION) 			{ responseByteSingle(0x03);				}
			else														{ responseByteSingle(0x00);				}
		} else if(cmdChar == CMD__STK_SET_DEVICE) {
			/* Setting the device is not supported. It's always an AVR .... */
			for(int i = 0; i < 20; i=i+1) { i2cReceiveNextByte(); }
			responseEmpty();
		} else if(cmdChar == CMD__STK_SET_DEVICE_EXT) {
			for(int i = 0; i < 5; i=i+1) { i2cReceiveNextByte(); }
			responseEmpty();
		} else if(cmdChar == CMD__STK_ENTER_PROGRAMMING_MODE) {
			responseEmpty(); /* Ignore entering programming mode - we program page by page anyways ... */
		} else if(cmdChar == CMD__STK_ERASE_DEVICE) {
			responseEmpty(); /* Ignore erase because we have a pagewise write mode */
		} else if(cmdChar == CMD__STK_LEAVE_PROGRAMMING_MODE) {
			responseEmpty();
			reset(); /* Reset device via watchdog method ... this will NOT enter bootloader after reboot! */
		} else if(cmdChar == CMD__STK_UNIVERSAL) {
			volatile uint8_t b0;
			volatile uint8_t b1;
			volatile uint8_t b2;
			volatile uint8_t b3;
			b0 = i2cReceiveNextByte();
			b1 = i2cReceiveNextByte();
			b2 = i2cReceiveNextByte();
			b3 = i2cReceiveNextByte();
			if(b0 == 0x30) {
				if(b2 == 0x00) { responseByteSingle(SIGNATUREBYTE_1); }
				else if(b2 == 0x01) { responseByteSingle(SIGNATUREBYTE_2); }
				else { responseByteSingle(SIGNATUREBYTE_3); }
			} else {
				responseByteSingle(0x00);
			}
		} else if(cmdChar == CMD__STK_LOAD_ADDRESS) {
			/* Loads a 16 bit address for FLASH or EEPROM access */
			uint8_t b0 = i2cReceiveNextByte();
			uint8_t b1 = i2cReceiveNextByte();
			address = (((uint16_t)b0)) | (((uint16_t)b1) << 8); /* Little endian encoding during transport */
			responseEmpty();
		} else if(cmdChar == CMD__STK_PROGPAGE) {
			/* Program a page */
			uint8_t b0 = i2cReceiveNextByte();
			uint8_t b1 = i2cReceiveNextByte();
			length = (((uint16_t)b0) << 8) | (((uint16_t)b1)); /* Big endian encoding! */
			int writeEeprom = 0;
			if(i2cReceiveNextByte() == 0x45) {
				writeEeprom = 1;
			}

			/* Now buffer the page ... */
			for(int i = 0; i < length; i=i+1) {
				buffer[i] = i2cReceiveNextByte();
			}

			if(i2cReceiveNextByte() == 0x20) {
				if(writeEeprom != 0) {
					/* Write into EEPROM page */
					address = address << 1;
					for(int i = 0; i < length; i=i+1) {
						/* Writing bytewise */
						while(EECR & (1 << EEPE)) { } /* Busy wait till we can write */
						EEAR = address;
						EEDR = buffer[i];
						EECR = EECR | (1 << EEMPE);
						EECR = EECR | (1 << EEPE);
					}
				} else {
					/* Write into FLASH page */
					#if 0
						RAMPZ = (address > 0x7FFF) ? 1 : 0;
					#endif
					address = address << 1; /* This masks off the uppermost bit - because of this RAMPZ is required */
					if((length & 0x0001) != 0) { length = length + 1; } /* Only use even lengths */

					cli();
					while(EECR & (1 << EEPE)) { } /* Busy wait for previous EEPROM write to finish before touching flash */

					asm volatile(
							"mov    r1, r28							\n\t"
							"mov    r2, r29							\n\t"
							"clr	r17								\n\t"
							"lds	r30,address						\n\t"
							"lds	r31,address+1					\n\t"
							"ldi	r28,lo8(buffer)					\n\t"
							"ldi	r29,hi8(buffer)					\n\t"
							"lds	r24,length						\n\t"
							"lds	r25,length+1					\n\t"
						"blockLoop:									\n\t"
							"cpi	r17,0x00						\n\t"
							"brne	skiperase						\n\t"
						"wait1:										\n\t"
							"lds	r16,%0							\n\t"
							"andi	r16,1           				\n\t"
							"cpi	r16,1           				\n\t"
							"breq	wait1		       				\n\t"
							"ldi	r16,0x03						\n\t"
							"sts	%0,r16							\n\t"
							"spm									\n\t"
						"wait2:										\n\t"
							"lds	r16,%0							\n\t"
							"andi	r16,1           				\n\t"
							"cpi	r16,1           				\n\t"
							"breq	wait2 		      				\n\t"
							"ldi	r16,0x11						\n\t"
							"sts	%0,r16							\n\t"
							"spm									\n\t"
						"skiperase:									\n\t"
							"ld		r0,Y+							\n\t"
							"ld		r1,Y+							\n\t"
						"wait3:										\n\t"
							"lds	r16,%0							\n\t"
							"andi	r16,1           				\n\t"
							"cpi	r16,1           				\n\t"
							"breq	wait3 	     					\n\t"
							"ldi	r16,0x01						\n\t"
							"sts	%0,r16							\n\t"
							"spm									\n\t"
							"inc	r17								\n\t"
							"cpi r17,%1	        					\n\t"
							"brlo	samepage						\n\t"
						"writepg:									\n\t"
							"clr	r17								\n\t"
						"wait4:										\n\t"
							"lds	r16,%0							\n\t"
							"andi	r16,1           				\n\t"
							"cpi	r16,1           				\n\t"
							"breq	wait4 		      				\n\t"
							"ldi	r16,0x05						\n\t"
							"sts	%0,r16							\n\t"
							"spm									\n\t"
						"wait5:										\n\t"
							"lds	r16,%0							\n\t"
							"andi	r16,1           				\n\t"
							"cpi	r16,1           				\n\t"
							"breq	wait5 		      				\n\t"
							"ldi	r16,0x11						\n\t"
							"sts	%0,r16							\n\t"
							"spm									\n\t"
						"samepage:									\n\t"
							"adiw	r30,2							\n\t"
							"sbiw	r24,2							\n\t"
							"breq	final							\n\t"
							"rjmp	blockLoop						\n\t"
						"final:										\n\t"
							"cpi	r17,0							\n\t"
							"breq	blockDone						\n\t"
							"adiw	r24,2							\n\t"
							"rjmp	writepg							\n\t"
						"blockDone:									\n\t"
							"clr	__zero_reg__					\n\t"
							"mov    r28, r1							\n\t"
							"mov    r29, r2							\n\t"

						: "=m" (SPMCSR) : "M" (PAGESIZE) : "r0","r1","r2","r16","r17","r24","r25","r30","r31"
					);
				}
				/* Signal page write (EEPROM or FLASH) has succeeded */
				i2cTransmitNextByte(0x14);
				i2cTransmitNextByte(0x10);
			} else {
				if((errorCounter = errorCounter + 1) > MAX_ERROR_COUNT) {
					/*
						In case we have received more errornous messages than
						MAX_ERROR_COUNT we simply leave the bootloader and
						boot the application as usual.
					*/
					enterApplication();
				}
			}
		} else if(cmdChar == CMD__STK_READPAGE) {
			/* Read a page for verification */
			uint8_t b0 = i2cReceiveNextByte();
			uint8_t b1 = i2cReceiveNextByte();
			length = (((uint16_t)b0) << 8) | (((uint16_t)b1)); /* Big endian encoding! */
			#if 0
				int nearFarRampz = ((address & 0x8000) != 0) ? 1 : 0;
			#endif
			int readEeprom = 0;
			if(i2cReceiveNextByte() == 0x45) {
				readEeprom = 1;
			}
			address = address << 1;

			if(i2cReceiveNextByte() == 0x20) {
				i2cTransmitNextByte(0x14);
				for(int i = 0; i < length; i++) {
					if(readEeprom != 0) {
						while(EECR & (1 << EEPE)) { }
						EEAR = address;
						EECR = EECR | (1 << EERE);
						i2cTransmitNextByte(EEDR);
						address = address + 1;
					} else {
						i2cTransmitNextByte(pgm_read_byte_near(address));
						#if 0
							if(nearFarRampz == 0) {
								i2cTransmitNextByte(pgm_read_byte_near(address));
							} else {
								i2cTransmitNextByte(pgm_read_byte_far(address + 0x10000));
							}
						#endif
						address = address + 1;
					}
				}
				i2cTransmitNextByte(0x10);
			}
		} else if(cmdChar == CMD__STK_READ_SIGNATURE) {
			if(i2cReceiveNextByte() == 0x20) {
				i2cTransmitNextByte(0x14);
				i2cTransmitNextByte(SIGNATUREBYTE_1);
				i2cTransmitNextByte(SIGNATUREBYTE_2);
				i2cTransmitNextByte(SIGNATUREBYTE_3);
				i2cTransmitNextByte(0x10);
			} else {
				if((errorCounter = errorCounter + 1) > MAX_ERROR_COUNT) {
					/*
						In case we have received more errornous messages than
						MAX_ERROR_COUNT we simply leave the bootloader and
						boot the application as usual.
					*/
					enterApplication();
				}
			}
		} else if(cmdChar == CMD__STK_READ_OSCCAL) {
			responseByteSingle(0x00);
		} else {
			if((errorCounter = errorCounter + 1) > MAX_ERROR_COUNT) {
				/*
					In case we have received more errornous messages than
					MAX_ERROR_COUNT we simply leave the bootloader and
					boot the application as usual.
				*/
				enterApplication();
			}
		}
	}
}
