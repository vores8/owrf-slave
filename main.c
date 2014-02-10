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
#include "owlib.h"

#include "debugprint.h"

#define NODENUM 0

static uint8_t transfer_buf[64];

int doSlave(uint8_t* message) {
	int i;
	uint8_t cmd = message[0];
	uint8_t datalen = 0;
	uint8_t retval = 0;
	if (cmd == 0x10) {
		debugPrint("cmd=PING retval=%02X\r\n", retval);
	} else if (cmd == 0x11) {
		transfer_buf[2] = OWFirst() ? 1 : 0;
		datalen = 1;
		debugPrint("cmd=FINDFIRSTDEVICE retval=%02X\r\n", retval);
	} else if (cmd == 0x12) {
		transfer_buf[2] = OWNext() ? 1 : 0;
		datalen = 1;
		debugPrint("cmd=FINDNEXTDEVICE retval=%02X\r\n", retval);
	} else if (cmd == 0x13) {
		datalen = 8;
		for (i = 0; i < 8; i++) {
			transfer_buf[i + 2] = getROM_NO(i);
		}
		debugPrint("cmd=GETADDRESS retval=%02X\r\n", retval);
	} else if (cmd == 0x16) {
		OWTargetSetup(0x00);
		debugPrint("cmd=SETSEARCHALLDEVICES retval=%02X\r\n", retval);
	} else if (cmd == 0x19) {
		transfer_buf[2] = OWReadByte();
		datalen = 1;
		debugPrint("cmd=READBYTE retval=%02X\r\n", retval);
	} else if (cmd == 0x1B) {
		uint8_t len;
		uint8_t* _buf;
		len = message[1];
		_buf = message + 2;
		retval = OWBlock(_buf, len) ? 0x00 : 0xFF;
		memcpy(transfer_buf + 2, _buf, len);
		datalen = len;
		debugPrint("cmd=DATABLOCK retval=%02X\r\n", retval);
	} else if (cmd == 0x1C) {
		transfer_buf[2] = OWReset();
		datalen = 1;
		debugPrint("cmd=RESET retval=%02X\r\n", retval);
	} else if (cmd == 0x1D) {
		OWWriteByte(message[1]);
		debugPrint("cmd=WRITEBYTEPOWER retval=%02X\r\n", retval);
	} else if (cmd == 0x1E) {
		OWLevel(MODE_NORMAL);
		debugPrint("cmd=SETPOWERNORMAL retval=%02X\r\n", retval);
	} else if (cmd == 0x1F) {
		OWTargetSetup(message[1]);
		debugPrint("cmd=TARGETFAMILYSETUP retval=%02X\r\n", retval);
	} else if (cmd == 0x20) {
		OWFamilySkipSetup();
		debugPrint("cmd=TARGETALLFAMILIES retval=%02X\r\n", retval);
	} else if (cmd == 0x21) {
		uint8_t address[8];
		memcpy(address, message + 2, 8);
		transfer_buf[2] = OWSearchROM(address);
		datalen = 1;
		debugPrint("cmd=SEARCHROM retval=%02X\r\n", retval);
	} else if (cmd == 0x22) {
		uint8_t address[8];
		memcpy(address, message + 2, 8);

		transfer_buf[2] = OWMatchROM(address); // ? 1 : 0;
		datalen = 1;
		debugPrint("cmd=MATCHROM retval=%02X\r\n", retval);
	} else {
		retval = 0xFF;
		debugPrint("default\r\n");
	}
	transfer_buf[1] = retval;
	if (retval == 0xFF) {
		return 1;
	}
	return datalen + 2;
}

int bytesRead = 0;
uint8_t channel = -1;
uint8_t* msg;
int phase = 0;

int main(void) {

	char* node_address = "OWRF1";
	node_address[4] += NODENUM;

	msg = transfer_buf;

	uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(57600, F_CPU));
	setPins(0x10);

	nrf24l01_init();
	nrf24l01_settxaddr((uint8_t*) "OWRF0");
	nrf24l01_setrxaddr(0, (uint8_t*) node_address);

	nrf24l01_enabledynamicpayloads();

	sei();

	for (;;) {
		uint8_t pipe = 0;
		if (nrf24l01_readready(&pipe)) {
			uint8_t len = nrf24l01_getdynamicpayloadsize();
			unsigned char message[32];
			debugPrint("got %d from %d\r\n", len, pipe);
//			nrf24l01_settxaddr((uint8_t*) "OWRF1");
			nrf24l01_read(transfer_buf, len);
			for (int i = 0; i < len; i++) {
				message[i] = transfer_buf[i];
				debugPrint(">%02X", message[i]);
			}
			debugPrint("\r\n");
			len = doSlave(message);
			_delay_ms(2);
			transfer_buf[0] = NODENUM;
			debugPrint("send %d\r\n", len);
			for (int i = 0; i < len; i++) {
				debugPrint("<%02X", transfer_buf[i]);
			}
			debugPrint("\r\n");
			uint8_t writeret = 0;
			nrf24l01_setrxaddr(0, (uint8_t*) "OWRF0");
			writeret = nrf24l01_write(transfer_buf, len);
			nrf24l01_setrxaddr(0, (uint8_t*) node_address);
			if (writeret == 1) {
				debugPrint("OK\r\n");
			} else {
				debugPrint("FAIL\r\n");
			}

		}
	}
}

