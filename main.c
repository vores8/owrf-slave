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

//#define UART_BAUD_SELECT(baudRate, xtalCpu)   ((xtalCpu)/((baudRate)*16l)-1)
//#define B9600 UART_BAUD_SELECT(9600, F_CPU)
//
//void uart_init( void )
//{
//  //настройка скорости обмена
//	  UBRR0H = (unsigned char) (B9600>>8);
//	  UBRR0L = (unsigned char) B9600;
//
//  //8 бит данных, 1 стоп бит, без контроля четности
//  UCSR0C = ( 1 << USBS0 ) | ( 1 << UCSZ01 ) | ( 1 << UCSZ00 );
//
//  //разрешить прием и передачу данных
//  UCSR0B = ( 1 << TXEN0 ) | ( 1 <<RXEN0 );
//}
//
//unsigned char uart_getc( void )
//{
//   //ждем приема байта
//   while( ( UCSR0A & ( 1 << RXC0 ) ) == 0  );
//   //считываем принятый байт
//   return UDR0;
//}
//
//void uart_putc( char c )
//{
//  //ждем окончания передачи предыдущего байта
//  while( ( UCSR0A & ( 1 << UDRE0 ) ) == 0 );
//  UDR0 = c;
//}


static uint8_t transfer_buf[64];

int doSlave(uint8_t* message) {
	int i;
	uint8_t cmd = message[0];
	uint8_t datalen = 0;
	uint8_t retval = 0;
	if (cmd == 0x10) {
		debugPrint("cmd=PING retval=%02X\r\n", retval);
	} else if (cmd == 0x11) {
		transfer_buf[1] = OWFirst() ? 1 : 0;
		datalen = 1;
		debugPrint("cmd=FINDFIRSTDEVICE retval=%02X\r\n",
				retval);
	} else if (cmd == 0x12) {
		transfer_buf[1] = OWNext() ? 1 : 0;
		datalen = 1;
		debugPrint("cmd=FINDNEXTDEVICE retval=%02X\r\n",
				retval);
	} else if (cmd == 0x13) {
		datalen = 8;
		for (i = 0; i < 8; i++) {
			transfer_buf[i + 1] = getROM_NO(i);
		}
		debugPrint("cmd=GETADDRESS retval=%02X\r\n",
				retval);
	} else if (cmd == 0x16) {
		OWTargetSetup(0x00);
		debugPrint("cmd=SETSEARCHALLDEVICES retval=%02X\r\n",
				retval);
	} else if (cmd == 0x19) {
		transfer_buf[1] = OWReadByte();
		datalen = 1;
		debugPrint("cmd=READBYTE retval=%02X\r\n",
				retval);
	} else if (cmd == 0x1B) {
		uint8_t len;
		uint8_t* _buf;
		len = message[1];
		_buf = message + 2;
		retval = OWBlock(_buf, len) ? 0x00 : 0xFF;
		memcpy(transfer_buf + 1, _buf, len);
		datalen = len;
		debugPrint("cmd=DATABLOCK retval=%02X\r\n",
				retval);
	} else if (cmd == 0x1C) {
		transfer_buf[1] = OWReset();
		datalen = 1;
		debugPrint("cmd=RESET retval=%02X\r\n",
				retval);
	} else if (cmd == 0x1D) {
		OWWriteByte(message[1]);
		debugPrint("cmd=WRITEBYTEPOWER retval=%02X\r\n",
				retval);
	} else if (cmd == 0x1E) {
		OWLevel(MODE_NORMAL);
		debugPrint("cmd=SETPOWERNORMAL retval=%02X\r\n",
				retval);
	} else if (cmd == 0x1F) {
		OWTargetSetup(message[1]);
		debugPrint("cmd=TARGETFAMILYSETUP retval=%02X\r\n",
				retval);
	} else if (cmd == 0x20) {
		OWFamilySkipSetup();
		debugPrint("cmd=TARGETALLFAMILIES retval=%02X\r\n",
				retval);
	} else if (cmd == 0x21) {
		uint8_t address[8];
		memcpy(address, message + 2, 8);
		transfer_buf[1] = OWSearchROM(address);
		datalen = 1;
		debugPrint("cmd=SEARCHROM retval=%02X\r\n",
				retval);
	} else if (cmd == 0x22) {
		uint8_t address[8];
		memcpy(address, message + 2, 8);

		transfer_buf[1] = OWMatchROM(address);// ? 1 : 0;
		datalen = 1;
		debugPrint("cmd=MATCHROM retval=%02X\r\n",
				retval);
	} else {
		retval = 0xFF;
		debugPrint("default\r\n");
	}
	transfer_buf[0] = retval;
	if (retval == 0xFF) {
		return 1;
	}
//	message[2] = datalen;
	return datalen + 1;
}

int bytesRead = 0;
uint8_t channel = -1;
uint8_t* msg;
int phase = 0;

int main(void) {

	msg = transfer_buf;

	uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(57600, F_CPU));
	setPins(0x10);

//	nrf24l01_init();
//	uint8_t addrtx0[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP0;
//	nrf24l01_settxaddr(addrtx0);
//	nrf24l01_enabledynamicpayloads();

	sei();

	for (;;) {
		int databyte;
		switch (phase) {
		case 0: {
			if (uart_available()) {
				databyte = uart_getc();
				if (bytesRead == 0) {
					if (databyte == 0xDD)
					transfer_buf[bytesRead++] = databyte;
				} else {
					transfer_buf[bytesRead++] = databyte;
				}
			}
			if (bytesRead == 4) {
				channel = transfer_buf[1];
				msg = transfer_buf + 2;
				debugPrint(
						"received command channel=%02X cmd=%02X len=%02X\r\n",
						channel, msg[0], msg[1]);
				bytesRead = 0;
				if (msg[1] == 0) {
					phase = 4;
				} else {
					phase = 3;
				}
			}
		}
			break;
		case 3: {
			if (bytesRead == msg[1]) {
				phase = 4;
			} else {
				if (uart_available() > 0) {
					databyte = uart_getc();
					*(msg + bytesRead + 2) = databyte;
					bytesRead++;
				}
			}
		}
			break;
		case 4: {
			if (channel >= 0) {
				uint8_t tosend[32];
				memcpy(tosend, msg, msg[1]+2);
				for (int i = 0; i < msg[1]+2; i++) {
					debugPrint(">%02X", tosend[i]);
				}
				debugPrint("\r\n");
				int len = doSlave(tosend);
				uart_putc(channel);
//				uart_putc(0);
//				debugPrint("outlen=%d\r\n", len);
				for (int i = 0; i < len; i++) {
					uart_putc(transfer_buf[i]);
					debugPrint("<%02X", transfer_buf[i]);
				}
//				debugPrint("\r\n");
				uart_flush();
			}
			phase = 0;
			bytesRead = 0;
			msg = transfer_buf;
		}
			break;
		}

	}
}

