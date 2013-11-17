/*
 * tempsens.c
 *
 *  Created on: 12.11.2013
 *      Author: uli
 */

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include "stdlib.h"
#include <stdint.h>
#include "tempsens.h"
#include <string.h>

#define W1_DIR "/sys/devices/w1_bus_master1/"
#define S_COUNT "w1_master_slave_count"
#define S_ADDRFILE "w1_master_slaves"

void get_sensor_addr(void) {
	int i, scount;
	uint8_t c;
	FILE *s;
	char fn[100];

	saddr[0][0] = '\0';
	saddr[1][0] = '\0';

	sprintf(fn, "%s%s", W1_DIR, S_COUNT);

	s = fopen(fn, "r");
	c = fgetc(s);
	scount = c - 48;
	//printf("scount: %i\n", scount);
	fclose(s);
	sprintf(fn, "%s%s", W1_DIR, S_ADDRFILE);
	s = fopen(fn, "r");
	for (i = 1; i <= scount; i++) {
		fgets(saddr[i - 1], 17, s);
		saddr[i - 1][15] = '\0';
		//printf("Sensor %i hat Seriennummer %s\n", i, saddr[i - 1]);
	}
	fclose(s);

}
int get_T(uint8_t sensor) {
	FILE *s;
	char fn[100];
	char data[41];
	char t[6];
	char crc[4];

	if (sensor >= 2) {
		return 999;
	} else {
		if (saddr[sensor][0] == '\0') {
			return 999;												// no sensor address available
		} else {
			sprintf(fn, "%s%s/w1_slave", W1_DIR, saddr[sensor]);
			s = fopen(fn, "r");
			fgets(data, 41, s);
			data[39] = '\0';
			//printf("%s\n", data);
			sprintf(crc, "%s", data+36);
			if (strcmp(crc, "YES") != 0) {
				fclose(s);
				return 999;											// crc error
			} else {
				fgets(data, 35, s);
				data[34] = '\0';
				//printf("%s\n", data);
				sprintf(t, "%s", data+29);
				//printf("T = %s grdC\n", t);
				fclose(s);
				return strtol(t, NULL, 10)/100;
			}
		}
	}
}
