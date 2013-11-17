/*
 * tempsens.h
 *
 *  Created on: 12.11.2013
 *      Author: uli
 */

#ifndef TEMPSENS_H_
#define TEMPSENS_H_

char saddr[2][17];
void get_sensor_addr(void);
int get_T(uint8_t sensor);

#endif /* TEMPSENS_H_ */
