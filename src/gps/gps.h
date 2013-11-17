/*
 * gps.h
 *
 *  Created on: 01.11.2013
 *      Author: uli
 */

#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>

#define GPS_ADDR 0x42

void startI2Cgps(void);
void setupGPS(void);
uint8_t gps_check_nav(void);
void gps_check_lock(uint8_t *lock, uint8_t *sats);
void gps_get_position(int *lon_int, int *lon_dec, int *lat_int, int *lat_dec, int32_t *alt);
void gps_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second);
void setGPS_NMEAoff(void);
void setGPS_DynamicModel6(void);
void setGPS_DynamicModel3(void);
void setGPS_PowerSaveMode(void);
void setGPS_MaxPerformanceMode(void);
void resetGPS(void);

#endif /* GPS_H_ */
