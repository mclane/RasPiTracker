/*
 * adc.c
 *
 *  Created on: 13.11.2013
 *      Author: uli
 */

#include <bcm2835.h>
#include <stdio.h>
#include "adc.h"

uint8_t adc_init(void) {

	if (!bcm2835_init()) {
		printf("oops, could not init bcm2835\n");
		return 1;
	} else {
		return 0;
	}
}

uint16_t adc_getV(int c) {

	uint16_t v;

	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);    // ~ 4 MHz
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default

	uint8_t mosi[10] = { 0x60, 0x00 };
	uint8_t miso[10] = { 0 };
	bcm2835_spi_transfernb(mosi, miso, 2);
	v = miso[1] + ((miso[0] & 3) << 8);
	//printf("Analogue level from SPI: %04x\n", miso[1] + ((miso[0] & 3) << 8));

	bcm2835_spi_end();
	//bcm2835_close();
	return v;
}
