/*
 * gps.c
 *
 *  Created on: 01.11.2013
 *      Author: uli
 */

#include "gps.h"
#include "stdio.h"
#include <stdlib.h>
#include "bcm2835_i2cbb.h"
#include "bcm2835.h"
#include <time.h>



uint8_t getUBX_ACK(uint8_t *MSG);

void startI2Cgps(void) {

	gpsi2c.address = GPS_ADDR;
	bcm2835_i2cbb_open(&gpsi2c, GPS_ADDR, 2, 3, 300, 2000000);

}

uint16_t fromGPS(void) {

	uint8_t dd[2];
	uint16_t d, i;
	uint32_t j;

	d = 0;
	i = 0;
	while (d == 0) {
		bcm8235_i2cbb_putc(&gpsi2c, 0xFD);
		for (j = 0; j< 200000; j++);
		bcm8235_i2cbb_gets(&gpsi2c, dd, 2);
		d = (dd[0] << 8) | dd[1];
		i++;
		if (i > 500) return 0;
	}
	return d;
}


void setupGPS() {
	//resetGPS();
	// Switch off all unwanted messages; change to polling mode
	//delay(500);
	setGPS_NMEAoff();
	delay(500);
	// switch on high altitude option
	setGPS_DynamicModel3();
	setGPS_MaxPerformanceMode();
	printf("DM3 on\n");
	delay(500);
}

uint8_t gps_check_nav(void) {

	// check gps navigation mode (needs to return 6 for high altitude mode)

	uint8_t request[] = { 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 };
	uint8_t data[45];
	uint16_t len;

	bcm8235_i2cbb_puts(&gpsi2c, request, 8);
	delay(50);
	// Get the message back from the GPS
	len = fromGPS();
	//printf("nav mode returned %i bytes \n", len);
	bcm8235_i2cbb_gets(&gpsi2c, data, 44);
	getUBX_ACK(request);
	// Return the navigation mode and let the caller analyse it
	return (uint8_t) *(data + 8);
}

void gps_check_lock(uint8_t *lock, uint8_t *sats) {

	// get lock status and sat number from gps

	// Construct the request to the GPS
	uint8_t request[8] = { 0xB5, 0x62, 0x01, 0x06, 0x00, 0x00, 0x07, 0x16 };
	uint8_t data[61];
	uint16_t len;

	bcm8235_i2cbb_puts(&gpsi2c, request, 8);
	delay(50);
	// Get the message back from the GPS
	len = fromGPS();
	//printf("checklock returned %i bytes\n", len);
	bcm8235_i2cbb_gets(&gpsi2c, data, 60);

	// Return the value if GPSfixOK is set in 'flags'
	if (*(data + 17) & 0x01)
		*lock = *(data + 16);
	else
		*lock = 0;

	*sats = *(data + 53);
}

void gps_get_position(int *lon_int, int *lon_dec, int *lat_int, int *lat_dec,
		int32_t *alt) {

	// get position and altitude from gps

	int32_t lon = 0, lat = 0, aalt = 0;
	uint8_t data[37];
	uint16_t len;

	// Request a NAV-POSLLH message from the GPS
	uint8_t request[8] = { 0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A };
	bcm8235_i2cbb_puts(&gpsi2c, request, 8);
	delay(50);
	// Get the message back from the GPS
	len = fromGPS();
	//printf("get_pos returned %i bytes\n", len);
	bcm8235_i2cbb_gets(&gpsi2c, data, 36);

	// 4 bytes of longitude (1e-7)
	lon = (int32_t) *(data + 10) | (int32_t) *(data + 11) << 8
			| (int32_t) *(data + 12) << 16 | (int32_t) *(data + 13) << 24;

	*lon_int = abs(lon / 10000000);
	*lon_dec = (labs(lon) % 10000000) / 100;
	// 4 bytes of latitude (1e-7)
	lat = (int32_t) *(data + 14) | (int32_t) *(data + 15) << 8
			| (int32_t) *(data + 16) << 16 | (int32_t) *(data + 17) << 24;

	*lat_int = abs(lat / 10000000);
	*lat_dec = (labs(lat) % 10000000) / 100;

	// 4 bytes of altitude above MSL (mm)
	aalt = (int32_t) *(data + 22) | (int32_t) *(data + 23) << 8
			| (int32_t) *(data + 24) << 16 | (int32_t) *(data + 25) << 24;
	*alt = aalt / 1000; // Correct to meters
}

void gps_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second) {

	// get UTC from gps

	// Send a NAV-TIMEUTC message to the receiver
	uint8_t request[8] = { 0xB5, 0x62, 0x01, 0x21, 0x00, 0x00, 0x22, 0x67 };
	uint8_t data[29];
	uint16_t len;

	bcm8235_i2cbb_puts(&gpsi2c, request, 8);
	delay(50);
	len = fromGPS();
	//printf("get_time returned %i bytes\n", len);
	bcm8235_i2cbb_gets(&gpsi2c, data, 28);

	*hour = *(data + 22);
	*minute = *(data + 23);
	*second = *(data + 24);
}

uint8_t getUBX_ACK(uint8_t *MSG) {
	uint8_t ubxi;
	uint8_t ackByteID = 0;
	uint8_t ackPacket[10];
	uint8_t data[10];
	uint16_t len = 0;

// Construct the expected ACK packet
	ackPacket[0] = 0xB5;	// header
	ackPacket[1] = 0x62;	// header
	ackPacket[2] = 0x05;	// class
	ackPacket[3] = 0x01;	// id
	ackPacket[4] = 0x02;	// length
	ackPacket[5] = 0x00;
	ackPacket[6] = MSG[2];	// ACK class
	ackPacket[7] = MSG[3];	// ACK id
	ackPacket[8] = 0;		// CK_A
	ackPacket[9] = 0;		// CK_B

// Calculate the checksums
	for (ubxi = 2; ubxi < 8; ubxi++) {
		ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
		ackPacket[9] = ackPacket[9] + ackPacket[8];
	}

	len = fromGPS();
	//printf("ack returned %i bytes\n", len);
	bcm8235_i2cbb_gets(&gpsi2c, data, 10);
	if (len < 10) {
		return 0;
	}
	for (ackByteID = 0; ackByteID <= 9; ackByteID++) {
		if (*(data + ackByteID) != ackPacket[ackByteID]) {
			return 0;
		}
	}
	//printf("success!\n");
	return 1;
}

void setGPS_NMEAoff(void) {
	int gps_set_sucess = 0;
	uint32_t len;
	uint8_t setNMEAoff[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
			0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xA6 };
	len = sizeof(setNMEAoff) / sizeof(uint8_t);
	//printf("enter NMEAoff\n");
	while (!gps_set_sucess) {
		bcm8235_i2cbb_puts(&gpsi2c, setNMEAoff, len);
		delay(500);
		gps_set_sucess = getUBX_ACK(setNMEAoff);
	}
	//printf("NMEA switched off\n");
}

void setGPS_DynamicModel6(void) {
	int gps_set_sucess = 0;
	uint32_t len;
	uint8_t setdm6[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
			0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00,
			0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x16, 0xDC };
	len = sizeof(setdm6) / sizeof(uint8_t);
	while (!gps_set_sucess) {
		bcm8235_i2cbb_puts(&gpsi2c, setdm6, len);
		delay(100);
		gps_set_sucess = getUBX_ACK(setdm6);
	}
	printf("Flight mode set\n");
}

void setGPS_DynamicModel3(void) {
	int gps_set_sucess = 0;
	uint32_t len;
	uint8_t setdm3[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
			0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00,
			0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x13, 0x76 };
	len = sizeof(setdm3) / sizeof(uint8_t);
	while (!gps_set_sucess) {
		bcm8235_i2cbb_puts(&gpsi2c, setdm3, len);
		delay(100);
		gps_set_sucess = getUBX_ACK(setdm3);
	}
}

void setGPS_PowerSaveMode(void) {
	// Power Save Mode
	int gps_set_sucess = 0;
	uint32_t len;
	uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22,
			0x92 }; // Setup for Power Save Mode (Default Cyclic 1s)
	len = sizeof(setPSM) / sizeof(uint8_t);
	while (!gps_set_sucess) {
		bcm8235_i2cbb_puts(&gpsi2c, setPSM, len);
		delay(100);
		gps_set_sucess = getUBX_ACK(setPSM);
	}
}

void setGPS_MaxPerformanceMode(void) {
	//Set GPS for Max Performance Mode
	int gps_set_sucess = 0;
	uint32_t len;
	uint8_t setMax[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21,
			0x91 }; // Setup for Max Power Mode
	len = sizeof(setMax) / sizeof(uint8_t);
	while (!gps_set_sucess) {
		bcm8235_i2cbb_puts(&gpsi2c, setMax, len);
		delay(100);
		gps_set_sucess = getUBX_ACK(setMax);
	}
}

void resetGPS(void) {
	int gps_set_sucess = 0;
	uint32_t len;
	uint8_t set_reset[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF,
			0x00, 0x00, 0x0C, 0x5D };
	printf("enter reset\n");
	len = sizeof(set_reset) / sizeof(uint8_t);
	while (!gps_set_sucess) {
		bcm8235_i2cbb_puts(&gpsi2c, set_reset, len);
		delay(500);
		gps_set_sucess = getUBX_ACK(set_reset);
		gps_set_sucess = 1;
	}
	printf("reset success\n");
}

