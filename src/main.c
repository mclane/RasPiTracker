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

//struct bcm2835_i2cbb ibb, adxl;
struct termios options;
int serial = -1;
int count = 1;
int istatus = 0, actual_pic = 0, req_img = 0;
int need_gps = 1;
int gps_available = 0;
int dom_f, rtty_f = 0;
struct txdata {
	char str[80];
	int len;
} gpsdata;

void *domex_tx(void) {
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
	printf("DomEX finished\n");
	dom_f = 1;
	return NULL;
}

void rtty_tx(void) {
	write(serial, gpsdata.str, gpsdata.len);
	tcdrain(serial);
	rtty_f = 1;
	//return NULL;
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

void *get_gps(void) {
	uint8_t lock = 0, sats = 0;				// lock status and # of sats in view
	uint8_t hour = 0, minute = 0, second = 0;				// time
	int lat_int = 0, lon_int = 0;							// position
	int32_t alt = 0;										// altitude
	int32_t lat_dec = 0, lon_dec = 0;
	uint8_t errorstatus = 0;								// error flags
	char txstring[80];										// transmit string
	uint8_t fm, gpserr;
	int tin, tout;
	uint16_t volt;
	FILE *dfile;

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

	sprintf(txstring,
			"$$$$$PYSY,%i,%02d:%02d:%02d,%i.%05d,%i.%05d,%d,%d,%i.%02d,%i.%i,%i.%i",
			count, hour, minute, second, lat_int, lat_dec, lon_int, lon_dec,
			alt, sats, volt / 1000, (volt % 1000) / 10, tin / 10,
			tin < 0 ? -tin % 10 : tin % 10, tout / 10,
			tout < 0 ? -tout % 10 : tout % 10);
	sprintf(txstring, "%s,%i", txstring, errorstatus);
	dfile = fopen("/home/pi/data.txt", "a");
	fprintf(dfile, "%s\n", txstring);
	fclose(dfile);
	sprintf(gpsdata.str, "%s*%04X\n", txstring, gps_CRC16_checksum(txstring));
	gpsdata.len = strlen(gpsdata.str);
	printf("%s", gpsdata.str);
	gps_available = 1;
	count++;

	return NULL;
}

void *run_camera(void) {
	char still[] =
			"raspistill -w 512 -h 288 -q 50 -rot 180 -o /home/pi/pics/im0000.jpg";
	char video[] =
			"raspivid -w 1280 -h 720 -b 8000000 -rot 180 -t 120000 -n -o /home/pi/vids/vid0000.h264";
	char hrstill[] = "raspistill -rot 180 -o /home/pi/pics/hrim0000.jpg";
	int img_id = 1, vid_id = 1, hr_img = 1;

	while (1) {
		if (req_img == 1) {
			sprintf(still,
					"raspistill -w 512 -h 288 -q 50 -rot 180 -o /home/pi/pics/im%04i.jpg",
					img_id);
			printf("Image file: %s\n", still);
			system(still);
			actual_pic = img_id;
			img_id++;
			req_img = 0;
		}
		sprintf(hrstill, "raspistill -rot 180 -o /home/pi/pics/hrim%04i.jpg",
				hr_img);
		printf("HDImage file: %s\n", hrstill);
		system(hrstill);
		hr_img++;
		sprintf(video,
				"raspivid -w 1280 -h 720 -b 8000000 -rot 180 -t 120000 -n -o /home/pi/vids/vid%04i.h264",
				vid_id);
		printf("Video file: %s\n", video);
		system(video);
		vid_id++;
	}

	return NULL;
}

uint16_t tx_image(uint16_t img_id) {
	int c, i;
	ssdv_t ssdv;
	uint8_t pkt[SSDV_PKT_SIZE], b[128];
	size_t r;
	pthread_t dom, rtty, ggps;
	pthread_attr_t attr;
	FILE *pfile;

	char pfname[] = "/home/pi/pics/im0000.jpg";
	sprintf(pfname, "/home/pi/pics/im%04i.jpg", actual_pic);

	req_img = 1;

	pfile = fopen(pfname, "rb");

	if (NULL == pfile) {
		printf("could not open %s\n", pfname);
	}

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	ssdv_enc_init(&ssdv, "PYSY", img_id);
	ssdv_enc_set_buffer(&ssdv, pkt);

	i = 0;

	while (1) {
		if (need_gps == 1) {
		pthread_create(&ggps, &attr, get_gps, NULL);
		need_gps = 0;
		}
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
		//printf("GPS: %i\n", gps_available);
		if (gps_available == 1) {
			if (rtty_f == 0 ) rtty_tx();
			if (dom_f == 0) pthread_create(&dom, &attr, domex_tx, NULL);

		}
		if ((dom_f == 1) && (rtty_f == 1)) {
			need_gps = 1;
			gps_available = 0;
			dom_f = 0;
			rtty_f = 0;
		}
	}

	printf("Wrote %i packets\n", i);
	fclose(pfile);
	return (i);
}

int main() {
	pthread_t camera, ggps;
	pthread_attr_t attr;

	int img_id = 1;

	startI2Cgps();
	setupGPS();
	//setGPS_MaxPerformanceMode();
	get_sensor_addr();
	adc_init();
	domex_setup();

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
	need_gps = 1;

	pthread_create(&camera, &attr, run_camera, NULL);

	while (1) {
		while (actual_pic == 0) {
		}
		tx_image(img_id);
		img_id++;

	}
	//tx_gps();
	//img_id++;

}
