/*
 * bcm2835_i2cbb.h
 *
 *  Created on: 02.11.2013
 *      Author: uli
 */

#ifndef BCM2835_I2CBB_H_
#define BCM2835_I2CBB_H_

#include <stdint.h>

struct bcm2835_i2cbb {
	uint8_t address; 		// 7 bit address
	uint8_t sda; 			// pin used for sda coresponds to gpio
	uint8_t scl; 			// clock
	uint32_t clock_delay; 	// proporional to bus speed
	uint32_t timeout;
};
int bcm2835_i2cbb_open(struct bcm2835_i2cbb *bb, uint8_t adr, 	// 7 bit address
		uint8_t data,   										// GPIO pin for data
		uint8_t clock,  										// GPIO pin for clock
		uint32_t speed, 										// clock delay 250 = 100KHz 500 = 50KHz (apx)
		uint32_t timeout);
void bcm8235_i2cbb_putc(struct bcm2835_i2cbb *bb, uint8_t value);
void bcm8235_i2cbb_puts(struct bcm2835_i2cbb *bb, uint8_t *s, uint32_t len);
uint8_t bcm8235_i2cbb_getc(struct bcm2835_i2cbb *bb);
void bcm8235_i2cbb_gets(struct bcm2835_i2cbb *bb, uint8_t *buf, uint32_t len);


#endif /* BCM2835_I2CBB_H_ */
