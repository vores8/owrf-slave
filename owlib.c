/*
 * owlib.cpp
 *
 *  Created on: 19 џэт. 2014 у.
 *      Author: dbkrasn
 */
/*
 * Main.cpp
 *
 * Created: 1/13/2014 12:34:20 PM
 *  Author: dbkrasn
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "compilers.h"

#include "polled/OWIPolled.h"
#include "polled/OWIHighLevelFunctions.h"
#include "polled/OWIBitFunctions.h"
#include "common_files\OWIcrc.h"

#include "owlib.h"

#include "debugprint.h"

unsigned char ROM_NO[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int LastDiscrepancy;
int LastFamilyDiscrepancy;
int LastDeviceFlag;
unsigned char crc8;

// DS2480B state
int ULevel; // 1-Wire level
int UBaud;  // baud rate
int UMode;  // command or data mode state
int USpeed; // 1-Wire communication speed
int ALARM_RESET_COMPLIANCE = FALSE; // flag for DS1994/DS2404 'special' reset
int ProgramAvailable;  // flag for 12-Volt program voltage available

unsigned char getROM_NO(int i) {
	return ROM_NO[i];
}

int currentBus = 0;
unsigned char BUSES = 0;
int numBuses = 0;
int _presence = 0;

unsigned char buses[5] = { OWI_PIN_3, OWI_PIN_4, OWI_PIN_5, OWI_PIN_6, OWI_PIN_7 };


void OWInit(int num) {
	BUSES = 0;
	numBuses = num > 5 ? 5 : num;
	for (int i = 0; i < numBuses; i++)
		BUSES |= buses[i];
	OWI_Init(BUSES);
}

//---------------------------------------------------------------------------
//-------- Basic 1-Wire functions
//---------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Reset all of the devices on the 1-Wire Net and return the result.
//
// Returns: TRUE(1):  presense pulse(s) detected, device(s) reset
//          FALSE(0): no presense pulses detected
//
// WARNING: Without setting the above global (FAMILY_CODE_04_ALARM_TOUCHRESET_COMPLIANCE)
//          to TRUE, this routine will not function correctly on some
//          Alarm reset types of the DS1994/DS1427/DS2404 with
//          Rev 1,2, and 3 of the DS2480/DS2480B.
//
//
int OWReset(void) {
	if (OWI_DetectPresence(BUSES)) {
		return TRUE;
	}
	// an error occured so re-sync with DS2480B
	DS2480B_Detect();

	return FALSE;
}

//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net.
// The parameter 'sendbit' least significant bit is used.
//
// 'sendbit' - 1 bit to send (least significant byte)
//
void OWWriteBit(unsigned char sendbit) {
	OWTouchBit(sendbit);
}

//--------------------------------------------------------------------------
// Send 8 bits of read communication to the 1-Wire Net and and return the
// result 8 bits read from the 1-Wire Net.
//
// Returns:  8 bits read from 1-Wire Net
//
unsigned char OWReadBit(void) {
	return OWTouchBit(0x01);
}

//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net and return the
// result 1 bit read from the 1-Wire Net.  The parameter 'sendbit'
// least significant bit is used and the least significant bit
// of the result is the return bit.
//
// 'sendbit' - the least significant bit is the bit to send
//
// Returns: 0:   0 bit read from sendbit
//          1:   1 bit read from sendbit
//
unsigned char OWTouchBit(unsigned char sendbit) {
	unsigned char readbuffer[10], sendpacket[10];
	unsigned char sendlen = 0;

	// make sure normal level
	OWLevel(MODE_NORMAL);

	// check for correct mode
	if (UMode != MODSEL_COMMAND) {
		UMode = MODSEL_COMMAND;
		sendpacket[sendlen++] = MODE_COMMAND;
	}

	// construct the command
	sendpacket[sendlen] = (sendbit != 0) ? BITPOL_ONE : BITPOL_ZERO;
	sendpacket[sendlen++] |= CMD_COMM | FUNCTSEL_BIT | USpeed;

	// send the packet
	if (WriteOWI(sendlen, sendpacket)) {
		// read back the response
		if (ReadOWI(1, readbuffer) == 1) {
			// interpret the response
			if (((readbuffer[0] & 0xE0) == 0x80)
					&& ((readbuffer[0] & RB_BIT_MASK) == RB_BIT_ONE))
				return 1;
			else
				return 0;
		}
	}

	// an error occured so re-sync with DS2480B
	DS2480B_Detect();

	return 0;
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and verify that the
// 8 bits read from the 1-Wire Net is the same (write operation).
// The parameter 'sendbyte' least significant 8 bits are used.
//
// 'sendbyte' - 8 bits to send (least significant byte)
//
// Returns:  TRUE: bytes written and echo was the same
//           FALSE: echo was not the same
//
void OWWriteByte(unsigned char sendbyte) {
//	OWTouchByte(sendbyte);
	unsigned char b = sendbyte;
	WriteOWI(1, &b);
}

//--------------------------------------------------------------------------
// Send 8 bits of read communication to the 1-Wire Net and and return the
// result 8 bits read from the 1-Wire Net.
//
// Returns:  8 bits read from 1-Wire Net
//
unsigned char OWReadByte(void) {
	debugPrint("bus=%X\r\n", currentBus);
//	return OWTouchByte(0xFF);
	unsigned char b;
	ReadOWI(1, &b);
	return b;
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and return the
// result 8 bits read from the 1-Wire Net.  The parameter 'sendbyte'
// least significant 8 bits are used and the least significant 8 bits
// of the result is the return byte.
//
// 'sendbyte' - 8 bits to send (least significant byte)
//
// Returns:  8 bits read from sendbyte
//
unsigned char OWTouchByte(unsigned char sendbyte) {
	unsigned char readbuffer[10], sendpacket[10];
	unsigned char sendlen = 0;

	// make sure normal level
//	OWLevel(MODE_NORMAL);

	// check for correct mode
//	if (UMode != MODSEL_DATA) {
//		UMode = MODSEL_DATA;
//		sendpacket[sendlen++] = MODE_DATA;
//	}

	// add the byte to send
	sendpacket[sendlen++] = (unsigned char) sendbyte;

	// check for duplication of data that looks like COMMAND mode
	if (sendbyte == (int) MODE_COMMAND)
		sendpacket[sendlen++] = (unsigned char) sendbyte;

	// send the packet
	if (WriteOWI(sendlen, sendpacket)) {
		// read back the 1 byte response
		if (ReadOWI(1, readbuffer) == 1) {
			// return the response
			return readbuffer[0];
		}
	}

	// an error occured so re-sync with DS2480B
	DS2480B_Detect();

	return 0;
}

//--------------------------------------------------------------------------
// The 'OWBlock' transfers a block of data to and from the
// 1-Wire Net. The result is returned in the same buffer.
//
// 'tran_buf' - pointer to a block of unsigned
//              chars of length 'tran_len' that will be sent
//              to the 1-Wire Net
// 'tran_len' - length in bytes to transfer

// Supported devices: all
//
// Returns:   TRUE (1) : The optional reset returned a valid
//                       presence (do_reset == TRUE) or there
//                       was no reset required.
//            FALSE (0): The reset did not return a valid prsence
//                       (do_reset == TRUE).
//
//  The maximum tran_length is (160)
//
int OWBlock(unsigned char *tran_buf, int tran_len) {

	debugPrint("OWBlock len=%d", tran_len);
	// check for a block too big
	if (tran_len > 160)
		return FALSE;

	// send the packet
	unsigned char pos = tran_len - 1;
	while (tran_buf[pos] == 0xFF) {
		pos--;
	}

	pos++;

	for (int i = 0; i < pos; i++) {
		OWI_SendByte(tran_buf[i], buses[currentBus]);
	}

	unsigned char ans = tran_len - pos;
	debugPrint(" ans=%d\r\n", ans);
	if (ans > 0) {
		for (int i = pos; i < tran_len; i++) {
			tran_buf[i] = OWI_ReceiveByte(buses[currentBus]);
		}
	}
	return TRUE;

	// an error occured so re-sync with DS2480B
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire bus
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : no device present
//
int OWFirst() {
	debugPrint("OWFirst\r\n");
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = FALSE;
	LastFamilyDiscrepancy = 0;
	currentBus = 0;
	_presence = OWI_DetectPresence(BUSES);

	return OWSearch();
}

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire bus
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
int OWNext() {
	// leave the search state alone
	return OWSearch();
}

//--------------------------------------------------------------------------
// Verify the device with the ROM number in ROM_NO buffer is present.
// Return TRUE  : device verified present
//        FALSE : device not present
//
int OWVerify() {
	unsigned char rom_backup[8];
	int i, rslt, ld_backup, ldf_backup, lfd_backup;

	// keep a backup copy of the current state
	for (i = 0; i < 8; i++)
		rom_backup[i] = ROM_NO[i];
	ld_backup = LastDiscrepancy;
	ldf_backup = LastDeviceFlag;
	lfd_backup = LastFamilyDiscrepancy;

	// set search to find the same device
	LastDiscrepancy = 64;
	LastDeviceFlag = FALSE;

	if (OWSearch()) {
		// check if same device found
		rslt = TRUE;
		for (i = 0; i < 8; i++) {
			if (rom_backup[i] != ROM_NO[i]) {
				rslt = FALSE;
				break;
			}
		}
	} else
		rslt = FALSE;

	// restore the search state
	for (i = 0; i < 8; i++)
		ROM_NO[i] = rom_backup[i];
	LastDiscrepancy = ld_backup;
	LastDeviceFlag = ldf_backup;
	LastFamilyDiscrepancy = lfd_backup;

	// return the result of the verify
	return rslt;
}

//--------------------------------------------------------------------------
// Setup the search to find the device type 'family_code' on the next call
// to OWNext() if it is present.
//
void OWTargetSetup(unsigned char family_code) {
	int i;

	// set the search state to find SearchFamily type devices
	ROM_NO[0] = family_code;
	for (i = 1; i < 8; i++)
		ROM_NO[i] = 0;
	LastDiscrepancy = 64;
	LastFamilyDiscrepancy = 0;
	currentBus = 0;
	LastDeviceFlag = FALSE;
}

//--------------------------------------------------------------------------
// Setup the search to skip the current device type on the next call
// to OWNext().
//
void OWFamilySkipSetup() {
	// set the Last discrepancy to last family discrepancy
	LastDiscrepancy = LastFamilyDiscrepancy;

	// clear the last family discrpepancy
	LastFamilyDiscrepancy = 0;

	// check for end of list
	if (LastDiscrepancy == 0) {
		LastDeviceFlag = TRUE;
		currentBus = 0;
	}

}

//--------------------------------------------------------------------------
// The 'OWSearch' function does a general search.  This function
// continues from the previos search state. The search state
// can be reset by using the 'OWFirst' function.
// This function contains one parameter 'alarm_only'.
// When 'alarm_only' is TRUE (1) the find alarm command
// 0xEC is sent instead of the normal search command 0xF0.
// Using the find alarm command 0xEC will limit the search to only
// 1-Wire devices that are in an 'alarm' state.
//
// Returns:   TRUE (1) : when a 1-Wire device was found and it's
//                       Serial Number placed in the global ROM
//            FALSE (0): when no new device was found.  Either the
//                       last search was the last device or there
//                       are no devices on the 1-Wire Net.
//
int OWSearch() {

	debugPrint("OWSearch LastDeviceFlag=%d presence=%x\r\n", LastDeviceFlag, _presence);

	if (LastDeviceFlag) {
		// reset the search
		LastDiscrepancy = 0;
		LastDeviceFlag = FALSE;
		LastFamilyDiscrepancy = 0;
		currentBus = 0;
		return FALSE;
	}

	while (!(_presence & buses[currentBus])) {
		currentBus++;
	}
	debugPrint("currentBus=%d\r\n");
	if (currentBus == numBuses) {
		LastDeviceFlag = TRUE;
		return FALSE;
	} else {

		OWI_DetectPresence(buses[currentBus]);
		LastDiscrepancy = OWI_SearchRom(ROM_NO, LastDiscrepancy,
				buses[currentBus]);
		if (LastDiscrepancy == OWI_ROM_SEARCH_FINISHED) {
				currentBus++;
		}
	}
	return OWI_CheckRomCRC(ROM_NO) == OWI_CRC_OK;
}

int OWSearchROM(unsigned char* address) {

	unsigned char tmp_rom[8];

	for (currentBus = 0; currentBus < numBuses; currentBus++) {
		if (OWI_DetectPresence(buses[currentBus])) {
			memcpy(tmp_rom, address, 8);
			OWI_SearchRom(tmp_rom, 64, buses[currentBus]);
			if (memcmp(address, tmp_rom, 8) == 0) {
				return TRUE;
			}
		}
	}
	return FALSE;
}

int OWMatchROM(unsigned char* address) {
	if (OWI_DetectPresence(buses[currentBus])) {
		_delay_ms(5);
		OWI_MatchRom(address, buses[currentBus]);
		return TRUE;
	}
	return FALSE;
//	OWI_DetectPresence(buses[currentBus]);
//	OWI_MatchRom(address, buses[currentBus]);
//	return TRUE;
}

void OWReadROM(unsigned char* address) {
	OWI_ReadRom(address, buses[currentBus]);
}

//---------------------------------------------------------------------------
//-------- DS2480B functions
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Attempt to resync and detect a DS2480B and set the FLEX parameters
//
// Returns:  TRUE  - DS2480B detected successfully
//           FALSE - Could not detect DS2480B
//
int DS2480B_Detect(void) {
	return TRUE;
}

//--------------------------------------------------------------------------
// Write an array of bytes to the COM port, verify that it was
// sent out.  Assume that baud rate has been set.
//
// 'outlen'   - number of bytes to write to COM port
// 'outbuf'   - pointer ot an array of bytes to write
//
// Returns:  TRUE(1)  - success
//           FALSE(0) - failure
//
int WriteOWI(int outlen, unsigned char *outbuf) {
	for (int i = 0; i < outlen; i++) {
		OWI_SendByte(outbuf[i], buses[currentBus]);
	}
	return TRUE;
}

//--------------------------------------------------------------------------
// Read an array of bytes to the COM port, verify that it was
// sent out.  Assume that baud rate has been set.
//
// 'inlen'     - number of bytes to read from COM port
// 'inbuf'     - pointer to a buffer to hold the incomming bytes
//
// Returns: number of characters read
//
int ReadOWI(int inlen, unsigned char *inbuf) {
//    while (!OWI_ReadBit(currentBus))
//    {
//
//    }
	for (int i = 0; i < inlen; i++) {
		inbuf[i] = OWI_ReceiveByte(buses[currentBus]);
	}
	return inlen;
}

//--------------------------------------------------------------------------
// Send a break on the com port for at least 2 ms
//

//---------------------------------------------------------------------------
//-------- Utility functions
//---------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Bit utility to read and write a bit in the buffer 'buf'.
//
// 'op'    - operation (1) to set and (0) to read
// 'state' - set (1) or clear (0) if operation is write (1)
// 'loc'   - bit number location to read or write
// 'buf'   - pointer to array of bytes that contains the bit
//           to read or write
//
// Returns: 1   if operation is set (1)
//          0/1 state of bit number 'loc' if operation is reading
//
int bitacc(int op, int state, int loc, unsigned char *buf) {
	int nbyt, nbit;

	nbyt = (loc / 8);
	nbit = loc - (nbyt * 8);

	if (op == WRITE_FUNCTION) {
		if (state)
			buf[nbyt] |= (0x01 << nbit);
		else
			buf[nbyt] &= ~(0x01 << nbit);

		return 1;
	} else
		return ((buf[nbyt] >> nbit) & 0x01);
}

// TEST BUILD
static unsigned char dscrc_table[] = { 0, 94, 188, 226, 97, 63, 221, 131, 194,

156, 126, 32, 163, 253, 31, 65, 157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227,

189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3,
		128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158,
		29, 67, 161, 255, 70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56,
		102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165,
		251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230, 167, 249,
		27, 69, 198, 152, 122, 36, 248, 166, 68, 26, 153, 199, 37, 123, 58, 100,
		134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179, 81, 15, 78, 16,
		242, 172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46, 204, 146, 211,
		141, 111, 49, 178, 236, 14, 80, 175, 241, 19, 77, 206, 144, 114, 44,
		109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208, 83, 13, 239,
		177, 240, 174, 76, 18, 145, 207, 45, 115, 202, 148, 118, 40, 171, 245,
		23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235, 181, 54, 104,
		138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183, 85, 11, 136,
		214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 116, 42, 200, 150,
		21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53 };

//--------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current
// global 'crc8' value.
// Returns current global crc8 value
//
unsigned char docrc8(unsigned char value) {
	// See Application Note 27

	// TEST BUILD
	crc8 = dscrc_table[crc8 ^ value];
	return crc8;
}

//--------------------------------------------------------------------------
// Set the 1-Wire Net line level.  The values for new_level are
// as follows:
//
// 'new_level' - new level defined as
//                MODE_NORMAL     0x00
//                MODE_STRONG5    0x02
//
// Returns:  current 1-Wire Net level
//
int OWLevel(int new_level) {
	unsigned char sendpacket[10], readbuffer[10];
	unsigned char sendlen = 0;
	unsigned char rt = FALSE, docheck = FALSE;

	// check if need to change level
	if (new_level != ULevel) {
		// check for correct mode
//		if (UMode != MODSEL_COMMAND) {
//			UMode = MODSEL_COMMAND;
//			sendpacket[sendlen++] = MODE_COMMAND;
//		}

		// check if just putting back to normal
		if (new_level == MODE_NORMAL) {
// check for disable strong pullup step
			if (ULevel == MODE_STRONG5)
				docheck = TRUE;

// stop pulse command
			sendpacket[sendlen++] = MODE_STOP_PULSE;

// add the command to begin the pulse WITHOUT prime
			sendpacket[sendlen++] = CMD_COMM | FUNCTSEL_CHMOD | SPEEDSEL_PULSE
					| BITPOL_5V | PRIME5V_FALSE;

// stop pulse command
			sendpacket[sendlen++] = MODE_STOP_PULSE;

// flush the buffers

// send the packet
			if (WriteOWI(sendlen, sendpacket)) {
				// read back the 1 byte response
				if (ReadOWI(2, readbuffer) == 2) {
// check response byte
					if (((readbuffer[0] & 0xE0) == 0xE0)
							&& ((readbuffer[1] & 0xE0) == 0xE0)) {
						rt = TRUE;
						ULevel = MODE_NORMAL;

					}
				}
			}
		}
		// set new level
		else {
// set the SPUD time value
			sendpacket[sendlen++] = CMD_CONFIG | PARMSEL_5VPULSE
					| PARMSET_infinite;
// add the command to begin the pulse
			sendpacket[sendlen++] = CMD_COMM | FUNCTSEL_CHMOD | SPEEDSEL_PULSE
					| BITPOL_5V;

// flush the buffers

// send the packet
			if (WriteOWI(sendlen, sendpacket)) {
				// read back the 1 byte response from setting time limit
				if (ReadOWI(1, readbuffer) == 1) {
// check response byte
					if ((readbuffer[0] & 0x81) == 0) {
						ULevel = new_level;
						rt = TRUE;
					}
				}
			}
		}

		// if lost communication with DS2480B then reset
		if (rt != TRUE)
			DS2480B_Detect();
	}

	// return the current level
	return ULevel;
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and verify that the
// 8 bits read from the 1-Wire Net is the same (write operation).
// The parameter 'sendbyte' least significant 8 bits are used.  After the
// 8 bits are sent change the level of the 1-Wire net.
//
// 'sendbyte' - 8 bits to send (least significant bit)
//
// Returns:  TRUE: bytes written and echo was the same, strong pullup now on
//           FALSE: echo was not the same
//
int OWWriteBytePower(int sendbyte) {
	unsigned char sendpacket[10], readbuffer[10];
	unsigned char sendlen = 0;
	unsigned char rt = FALSE;
	unsigned char i, temp_byte;

	// check for correct mode
	if (UMode != MODSEL_COMMAND) {
		UMode = MODSEL_COMMAND;
		sendpacket[sendlen++] = MODE_COMMAND;
	}

	// set the SPUD time value
	sendpacket[sendlen++] = CMD_CONFIG | PARMSEL_5VPULSE | PARMSET_infinite;

	// construct the stream to include 8 bit commands with the last one
	// enabling the strong-pullup
	temp_byte = sendbyte;
	for (i = 0; i < 8; i++) {
		sendpacket[sendlen++] = ((temp_byte & 0x01) ? BITPOL_ONE : BITPOL_ZERO)

		| CMD_COMM | FUNCTSEL_BIT | USpeed
				| ((i == 7) ? PRIME5V_TRUE : PRIME5V_FALSE);
		temp_byte >>= 1;
	}

	// send the packet
	if (WriteOWI(sendlen, sendpacket)) {
		// read back the 9 byte response from setting time limit
		if (ReadOWI(9, readbuffer) == 9) {
// check response
			if ((readbuffer[0] & 0x81) == 0) {
				// indicate the port is now at power delivery
				ULevel = MODE_STRONG5;

				// reconstruct the echo byte
				temp_byte = 0;
				for (i = 0; i < 8; i++) {
					temp_byte >>= 1;
					temp_byte |= (readbuffer[i + 1] & 0x01) ? 0x80 : 0;
				}

				if (temp_byte == sendbyte)
					rt = TRUE;
			}
		}
	}

	// if lost communication with DS2480B then reset
	if (rt != TRUE)
		DS2480B_Detect();

	return rt;
}

