/*
 * main.c
 *
 *  Created on: 01.11.2013
 *      Author: uli
 */

#include "gps.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include "bcm2835_i2cbb.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "ssdv.h"
#include "tempsens.h"
#include "adc.h"
#include "domino.h"
#include <pthread.h>
#include "gpio.h"
#include <sys/wait.h>
#include "adxl345.h"
#include "bmp085.h"
#include "hmc5883.h"
#include "rs6.h"

#define PAYLOAD 49
#define PARITY 16
#define HEAD 5
#define PCKT 255

//struct bcm2835_i2cbb ibb, adxl;
struct termios options;
int serial = -1;
int count = 1;
int req_img = 1, img_id = 0, actual_img = 0;
struct txdata {
	char str[80];
	char strec[PAYLOAD+PARITY+HEAD+1];
	int len;
	int new;
} gpsdata;
pthread_mutex_t gpsacc_mutex = PTHREAD_MUTEX_INITIALIZER;

void domex_tx(void) {
	int i;
	GPIO_SET = 1 << 23;	// Enable NTX2b
	for (i = 0; i <= 5; i++) {
		domex_txchar(13);
	}
	for (i = 0; i <= gpsdata.len; i++) {
		domex_txchar((uint16_t) gpsdata.str[i]);
	}
	domex_txchar(13);
	GPIO_CLR = 1 << 23;	// Disable NTX2b
	//printf("DomEX finished\n");
}

uint16_t gps_CRC16_checksum(char *string) {
	int i, j;
	uint16_t crc;
	uint8_t d;

	crc = 0xFFFF;

// Calculate checksum ignoring the first $s
	for (i = 5; i < strlen(string); i++) {
		d = (uint8_t) string[i];
		crc = crc ^ ((uint16_t) d << 8);
		for (j = 0; j < 8; j++) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
		}
	}

	return crc;
}

void get_gps(void) {
	uint8_t lock = 0, sats = 0;				// lock status and # of sats in view
	uint8_t hour = 0, minute = 0, second = 0;				// time
	int lat_int = 0, lon_int = 0;							// position
	int32_t alt = 0;										// altitude
	int32_t lat_dec = 0, lon_dec = 0;
	uint8_t errorstatus = 0;								// error flags
	char txstring[85];										// transmit string
	uint8_t fm, gpserr;
	int tin, tout;
	uint16_t volt;
	FILE *dfile;
	int i;
	uint8_t parity[PARITY];
	uint8_t data[PAYLOAD];

	gpserr = 0;
	//do {
	fm = gps_check_nav();
	//printf("GPS mode = %i\n", fm);
	gps_check_lock(&lock, &sats);
	//printf("lock = %i sats = %i\n", lock, sats);
	gps_get_position(&lon_int, &lon_dec, &lat_int, &lat_dec, &alt);
	gps_get_time(&hour, &minute, &second);
	gpserr++;
	//} while ((alt > 50000) || (hour >= 24) || (minute >= 60) || second >= 60
	//		|| sats > 15 || fm > 6);
	if (gpserr > 1) {
		errorstatus |= (1 << 4);
	} else {
		errorstatus &= ~(1 << 4);
	}
	if (lock == 3) {
		errorstatus &= ~(1 << 5);
	} else {
		errorstatus |= (1 << 5);
	}
	if (fm == 6) {
		errorstatus &= ~(1 << 1);
	} else {
		errorstatus |= (1 << 1);
	}
	do {
		tin = get_T(0);
	} while (tin == 999);
	do {
		tout = get_T(1);
	} while (tout == 999);
	volt = adc_getV(0);
	volt = 3300 * volt / 489;
	// Begin protected section
	pthread_mutex_lock(&gpsacc_mutex);

	sprintf(txstring,
			"$$$$$PYSY,%i,%02d:%02d:%02d,%i.%05d,%i.%05d,%d,%d,%i.%02d,%i.%i,%i.%i",
			count, hour, minute, second, lat_int, lat_dec, lon_int, lon_dec,
			alt, sats, volt / 1000, (volt % 1000) / 10, tin / 10,
			tin < 0 ? -tin % 10 : tin % 10, tout / 10,
			tout < 0 ? -tout % 10 : tout % 10);
	sprintf(txstring, "%s,%i", txstring, errorstatus);
	dfile = fopen("/home/pi/data.txt", "a");
	fprintf(dfile, "%s", txstring);
	fclose(dfile);
	sprintf(gpsdata.str, "%s*%04X\n", txstring, gps_CRC16_checksum(txstring));
	gpsdata.len = strlen(gpsdata.str);
	gpsdata.new = 1;
	printf("%s", gpsdata.str);

	sprintf(txstring,
			"PYSY%04i%02i%02i%02i%02i%05i%03i%05i%05i%02i%03i%04i%04i%02i",
			count, hour, minute, second, lat_int, lat_dec, lon_int, lon_dec,
			alt, sats, volt / 10, tin, tout, errorstatus);
	for(i=0; i<PAYLOAD; i++) data[i] = (uint8_t) txstring[i];
	encode_rs_6(&data[0], &parity[0], PCKT-PARITY-PAYLOAD);
	for(i=0; i<PARITY; i++) txstring[PAYLOAD+i] = (char) parity[i];
	txstring[PAYLOAD+PARITY+1] = '\0';
	sprintf(gpsdata.strec, "&&&&&%s", txstring);
	for(i=0; i<PAYLOAD; i++) printf("%c", txstring[i]);
	for(i=PAYLOAD; i<(PAYLOAD+PARITY); i++) printf(" %02X", txstring[i]);
	printf("\n");

	/*
	 pad = 223 - gpsdata.len;
	 encode_rs_8(&gpsdata.str[0], &gpsdata.parity[0], pad);
	 for (i = 0; i < 32; i++) printf("%02X", gpsdata.parity[i]);
	 printf("\n");
	 */
	// End protected section
	pthread_mutex_unlock(&gpsacc_mutex);

	count++;
}

void get_sensordata(void) {
	FILE *dfile;
	int16_t ax = 0, ay = 0, az = 0, mx = 0, my = 0, mz = 0;
	int32_t p = 0, t = 0;

	adxl345_xyz(&ax, &ay, &az);
	bmp085_get(&p, &t);
	hmc5883_xyz(&mx, &my, &mz);
	dfile = fopen("/home/pi/data.txt", "a");
	fprintf(dfile, ",%i,%i,%i,%i,%i,%i,%i.%i,%i.%i\n", ax, ay, az, mx, my, mz,
			p / 100, p % 100, t / 10, t < 0 ? -t % 10 : t % 10);
	fclose(dfile);
	printf(
			"ax = %i ay = %i az = %i\nmx= %i my = %i mz = %i\np = %i.%i t = %i.%i\n",
			ax, ay, az, mx, my, mz, p / 100, p % 100, t / 10,
			t < 0 ? -t % 10 : t % 10);
}
;

void *run_camera(void) {
	char still[] =
			"raspistill -w 512 -h 288 -q 50 -rot 180 -ex auto -o /home/pi/pics/im0000.jpg";
	char HDstill[] = "raspistill -rot 180 -ex auto -o /home/pi/pics/HD0000.jpg";
	char video[] =
			"raspivid -w 1280 -h 720 -b 8000000 -rot 180 -ex auto -t 120000 -n -o /home/pi/pics/vid0000.h264";
	int vid_id = 1, HDimg_id = 1;
	while (1) {
		if (req_img == 1) {
			img_id++;
			sprintf(still,
					"raspistill -w 512 -h 288 -q 50 -rot 180 -ex auto -o /home/pi/pics/im%04i.jpg",
					img_id);
			printf("Still im%04i.jpg started\n", img_id);
			system(still);
			actual_img = img_id;
			req_img = 0;
		}
		sprintf(HDstill, "raspistill -rot 180 -ex auto -o /home/pi/pics/HD%04i.jpg",
				HDimg_id);
		printf("HD still HD%04i.jpg started\n", HDimg_id);
		system(HDstill);
		HDimg_id++;
		sprintf(video,
				"raspivid -w 1280 -h 720 -b 8000000 -rot 180 -ex auto -t 120000 -n -o /home/pi/vids/vid%04i.h264",
				vid_id);
		printf("Video vid%04i.h264 started\n", vid_id);
		system(video);
		vid_id++;
	}
	return NULL;
}

void *tx_image(void) {
	int c, i;
	ssdv_t ssdv;
	uint8_t pkt[SSDV_PKT_SIZE], b[128];
	size_t r;
	FILE *pfile;
	char d;

	while (1) {

		char pfname[] = "/home/pi/pics/im0000.jpg";
		sprintf(pfname, "/home/pi/pics/im%04i.jpg", img_id);

		req_img = 1;
		pfile = fopen(pfname, "rb");

		if (NULL == pfile) {
			printf("could not open %s\n", pfname);
		}

		ssdv_enc_init(&ssdv, "PYSY", actual_img);
		ssdv_enc_set_buffer(&ssdv, pkt);

		i = 0;

		while (1) {
			while ((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME) {
				r = fread(b, 1, 128, pfile);
				if (r <= 0) {
					printf("Premature end of file\n");
					break;
				}

				ssdv_enc_feed(&ssdv, b, r);
			}

			if (c == SSDV_EOI) {
				printf("ssdv_enc_get_packet said EOI\n");
				break;
			} else if (c != SSDV_OK) {
				printf("ssdv_enc_get_packet failed: %i\n", c);
				fclose(pfile);
				return (0);
			}

			write(serial, pkt, SSDV_PKT_SIZE);
			tcdrain(serial);
			i++;

			pthread_mutex_lock(&gpsacc_mutex);
			if (gpsdata.new == 1) {
				write(serial, gpsdata.str, gpsdata.len);
				write(serial, gpsdata.strec, PAYLOAD+PARITY+HEAD);
				write(serial, "\0xff", 1);
				write(serial, "\0x00", 1);
				tcdrain(serial);
				//printf("RTTY finished\n");
				gpsdata.new = 0;
			}
			pthread_mutex_unlock(&gpsacc_mutex);
		}
		printf("Wrote %i packets\n", i);
		fclose(pfile);
	}
	return NULL;
}

int main() {

	pthread_t camera, image;
	pthread_attr_t attr;

	startI2Cgps();
	setupGPS();
	//setGPS_MaxPerformanceMode();
	get_sensor_addr();
	adc_init();
	domex_setup();

	setupADXL();
	setupBMP085();
	setupHMC();

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	serial = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	tcgetattr(serial, &options);
	options.c_cflag = B300 | CS8 | CLOCAL | CSTOPB;		//<Set baud rate
	options.c_iflag = 0;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(serial, TCIFLUSH);
	tcsetattr(serial, TCSANOW, &options);

	req_img = 1;
	gpsdata.new = 0;

	get_gps();
	pthread_create(&camera, &attr, run_camera, NULL);
	while (actual_img == 0)
		;
	pthread_create(&image, &attr, tx_image, NULL);

	while (1) {
		get_gps();
		domex_tx();
		get_sensordata();
		sleep(10);
	}
}
