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

void uart_read(unsigned char* buf, uint8_t len) {
	for (uint8_t i = 0; i < len; i ++) {
		*buf++ = uart_getc();
	}
}

void uart_write(unsigned char* buf, uint8_t len) {
	for (uint8_t i = 0; i < len; i ++) {
		uart_putc(*buf++);
	}
}



int doSlave(unsigned char* message) {
	int i;
	unsigned char cmd = message[1];
	int result = 0;
	unsigned char retval = 0;
	switch (cmd) {
	case 0x10: {
		debugPrint("DS2480B_Detect\r\n");
		retval = DS2480B_Detect() ? 0x00 : 0xff;
		debugPrint("DS2480B_Detect:%02x\r\n", retval);
		debugPrint("channel=%02X cmd=PING retval=%02X\r\n", message[0], retval);
	}
		break;
	case 0x11: {
		unsigned char r = OWFirst() ? 1 : 0;
		result = 1;
		message[3] = r;
		debugPrint("channel=%02X cmd=FINDFIRSTDEVICE retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x12: {
		unsigned char r = OWNext() ? 1 : 0;
		result = 1;
		message[3] = r;

		debugPrint("channel=%02X cmd=FINDNEXTDEVICE retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x13: {
		result = 8;
		for (i = 0; i < 8; i++) {
			message[i + 3] = getROM_NO(i);
		}
		debugPrint("channel=%02X cmd=GETADDRESS retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x16: {
		OWTargetSetup(0x00);
		debugPrint("channel=%02X cmd=SETSEARCHALLDEVICES retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x19: {
		message[3] = OWReadByte();
		result = 1;
		debugPrint("channel=%02X cmd=READBYTE retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x1B: {
		unsigned char len;
		unsigned char* _buf;
		len = message[2];
		_buf = message + 3;
		OWBlock(_buf, len);
		result = len;
		debugPrint("channel=%02X cmd=DATABLOCK retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x1C: {
		message[3] = OWReset();
		result = 1;
		debugPrint("channel=%02X cmd=RESET retval=%02X\r\n", message[0], retval);
	}
		break;
	case 0x1D: {
		OWWriteByte(message[3]);
		debugPrint("channel=%02X cmd=WRITEBYTEPOWER retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x1E: {
		OWLevel(MODE_NORMAL);
		debugPrint("channel=%02X cmd=SETPOWERNORMAL retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x1F: {
		OWTargetSetup(message[3]);
		debugPrint("channel=%02X cmd=TARGETFAMILYSETUP retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x20: {
		OWFamilySkipSetup();
		debugPrint("channel=%02X cmd=TARGETALLFAMILIES retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x21: {
		message[3] = OWSearchROM(message+3);
		result = 1;
		debugPrint("channel=%02X cmd=SEARCHROM retval=%02X\r\n", message[0],
				retval);
	}
		break;
	case 0x22: {
		message[3] = OWMatchROM(message+3);
		result = 1;
		debugPrint("channel=%02X cmd=SEARCHROM retval=%02X\r\n", message[0],
				retval);
	}
		break;
	default:
		debugPrint("default\r\n");
		break;
	}
	message[1] = retval;
	message[2] = result;
return result;
}


int bytesRead = 0;
unsigned char channel = -1;
unsigned char transfer_buf[32];
int phase = 0;

int main(void) {
	uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(9600, F_CPU));
	setPins(0x10);
	nrf24l01_init();
	sei();

	uint8_t addrtx0[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP0;
	nrf24l01_settxaddr(addrtx0);
	nrf24l01_enabledynamicpayloads();

	for (;;) {
		unsigned char databyte;
		switch (phase) {
		case 0: {
			if (uart_available()) {
				databyte = uart_getc();
				transfer_buf[bytesRead++] = databyte;
			}
			if (bytesRead == 4) {
				if (transfer_buf[0] == 0x0) {
					transfer_buf[0] = transfer_buf[1];
					transfer_buf[1] = transfer_buf[2];
					transfer_buf[2] = transfer_buf[3];
					bytesRead = 3;
				} else {
					bytesRead = 0;
					if (transfer_buf[0] == 0xff) {
						channel = transfer_buf[1];
						transfer_buf[0] = transfer_buf[1];
						transfer_buf[1] = transfer_buf[2];
						transfer_buf[2] = transfer_buf[3];
						debugPrint(
								"received command channel=%02X cmd=%02X len=%02X\r\n",
								channel, transfer_buf[1], transfer_buf[2]);
						if (transfer_buf[2] == 0) {
							phase = 4;
						} else {
							phase = 3;
						}
					} else {
						phase = 0;
					}
				}
			}
		}
			break;
		case 3: {
			if (bytesRead == transfer_buf[2]) {
				phase = 4;
			} else {
				if (uart_available() > 0) {
					databyte = uart_getc();
					transfer_buf[bytesRead + 3] = databyte;
					bytesRead++;
				}
			}
		}
			break;
		case 4: {
			if (channel >= 0) {
				doSlave(transfer_buf);
				int len = transfer_buf[2] + 3;
				for (int i = 0; i < len; i++) {
					uart_putc(transfer_buf[i]);
				}
				uart_flush();
//				unsigned char pipe[] = "OWRF1";
//				pipe[4] += channel;
////				radio.stopListening();
////				radio.closeReadingPipe(1);
////				radio.openReadingPipe(1, *(uint64_t*) pipe);
//				//			uart_print(transfer_buf[2] + 3);
//				//			uart_print(":");
//				nrf24l01_write(transfer_buf, transfer_buf[2] + 3);
//				_delay_ms(4);
////				radio.startListening();
//
//				for (int count = 0; count < 15; count++) {
//					if (nrf24l01_readready(&pipe)) {
//						int l = nrf24l01_getdynamicpayloadsize();
//						//					uart_print(l);
//						nrf24l01_read(transfer_buf, l);
//						uart_write(transfer_buf, l);
//						debugPrint(
//								"done for channel=%02X resp=%02X len=%02X\r\n",
//								transfer_buf[0], transfer_buf[1],
//								transfer_buf[2]);
//						//						break;
//					}
//					_delay_ms(15);
//				}

				// Wait here until we get a response, or timeout (250ms)
				//			unsigned long started_waiting_at = millis();
				//			bool timeout = false;
				//			while (!radio.available() && !timeout)
				//				if ((millis() - started_waiting_at)
				//						> (1 + radio.getMaxTimeout() / 1000))
				//					timeout = true;
				//			if (!timeout) {
				//				int l = radio.getDynamicPayloadSize();
				//				if (radio.read(&transfer_buf[0], l)) {
				//					uart_write(transfer_buf, l);
				//					debugPrint("done for channel=%02X resp=%02X len=%02X\r\n",
				//							transfer_buf[0], transfer_buf[1], transfer_buf[2]);
				//				}
				//			}

			}
			phase = 0;
			bytesRead = 0;
		}
			break;
		}

	}
}

