/*
 nrf24l01 lib sample

 copyright (c) Davide Gironi, 2012

 Released under GPLv3.
 Please refer to LICENSE file for licensing information.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <uart.h>

#include <nrf24l01.h>
#include <owlib.h>
#include <owrfreadwrite.h>

#include <debugprint.h>

//#include <ds1992.h>

#define NODENUM 0

uint8_t doSlave(uint8_t* message) {
	uint8_t i;
	uint8_t cmd = message[0];
	uint8_t datalen = 0;
	uint8_t retval = 0;
	debugPrint("command=%x\r\n", cmd);
	if (cmd == 0x10) {
	} else if (cmd == 0x11) {
		message[2] = OWFirst() ? 1 : 0;
		datalen = 1;
	} else if (cmd == 0x12) {
		message[2] = OWNext() ? 1 : 0;
		datalen = 1;
	} else if (cmd == 0x13) {
		datalen = 8;
		for (i = 0; i < 8; i++) {
			message[i + 2] = getROM_NO(i);
		}
	} else if (cmd == 0x16) {
		OWTargetSetup(0x00);
	} else if (cmd == 0x19) {
		message[2] = OWReadByte();
		datalen = 1;
	} else if (cmd == 0x1B) {
		uint8_t len;
		uint8_t* _buf;
		len = message[1];
		_buf = message + 2;
		retval = OWBlock(_buf, len) ? 0x00 : 0xFF;
		memcpy(message + 2, _buf, len);
		datalen = len;
	} else if (cmd == 0x1C) {
		message[2] = OWReset();
		datalen = 1;
	} else if (cmd == 0x1D) {
//		OWWriteByte(message[1]);
		uint8_t len;
		uint8_t* _buf;
		len = message[1];
		_buf = message + 2;
		retval = OWBlock(_buf, len) ? 0x00 : 0xFF;
		memcpy(message + 2, _buf, len);
		datalen = len;
	} else if (cmd == 0x1E) {
//		OWLevel(MODE_NORMAL);
	} else if (cmd == 0x1F) {
		OWTargetSetup(message[1]);
	} else if (cmd == 0x20) {
		OWFamilySkipSetup();
	} else if (cmd == 0x21) {
		uint8_t address[8];
		memcpy(address, message + 2, 8);
		message[2] = OWSearchROM(address);
		datalen = 1;
	} else if (cmd == 0x22) {
		uint8_t address[8];
		memcpy(address, message + 2, 8);
		message[2] = OWMatchROM(address);
		datalen = 1;
	} else {
		retval = 0xFF;
	}
	message[0] = retval;
	if (retval == 0xFF) {
		datalen = 0;
	}
	message[1] = datalen;
	return datalen;
}

static uint8_t transfer_buf[255];

int main(void) {

#ifdef DEBUGPRINT
	uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(57600, F_CPU));
#endif

	char* node_address = "OWRF1";
	node_address[4] += NODENUM;

	OWInit(5);

	nrf24l01_init();
	nrf24l01_settxaddr((uint8_t*) node_address);
	nrf24l01_setrxaddr(0, (uint8_t*) node_address);
	nrf24l01_setrxaddr(1, (uint8_t*) "OWRF0");

	sei();

	for (;;) {
		uint8_t pipe;
		if (nrf24l01_readready(&pipe)) {
			int len = owrf_read(transfer_buf);
			if (len != -1) {
#ifdef DEBUGPRINT
				debugPrint("channel=%d command=%X datalen=%d\r\n", transfer_buf[0], transfer_buf[1], transfer_buf[2]);
				for (int i = 0; i < transfer_buf[2]; i++) {
					debugPrint(">%02X", transfer_buf[i+3]);
				}
				debugPrint("\r\n");
#endif
				if (transfer_buf[0] == NODENUM) {
					len = doSlave(transfer_buf + 1) + 3;
#ifdef DEBUGPRINT
					debugPrint("retval=%X datalen=%d\r\n", transfer_buf[1], transfer_buf[2]);
					for (int i = 0; i < transfer_buf[2]; i++) {
						debugPrint("<%02X", transfer_buf[i+3]);
					}
					debugPrint("\r\n");
#endif
					_delay_ms(5);
					owrf_write(transfer_buf, len);
				}
			}
		}
	}
}

