/*
 * domino.c
 *
 *  Created on: 13.11.2013
 *      Author: uli
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include "gpio.h"

unsigned char varicode[][3] = {
/* Primary alphabet */
{ 1, 15, 9 }, { 1, 15, 10 }, { 1, 15, 11 }, { 1, 15, 12 }, { 1, 15, 13 }, { 1,
		15, 14 }, { 1, 15, 15 }, { 2, 8, 8 }, { 2, 12, 0 }, { 2, 8, 9 }, { 2, 8,
		10 }, { 2, 8, 11 }, { 2, 8, 12 }, { 2, 13, 0 }, { 2, 8, 13 },
		{ 2, 8, 14 }, { 2, 8, 15 }, { 2, 9, 8 }, { 2, 9, 9 }, { 2, 9, 10 }, { 2,
				9, 11 }, { 2, 9, 12 }, { 2, 9, 13 }, { 2, 9, 14 }, { 2, 9, 15 },
		{ 2, 10, 8 }, { 2, 10, 9 }, { 2, 10, 10 }, { 2, 10, 11 }, { 2, 10, 12 },
		{ 2, 10, 13 }, { 2, 10, 14 }, { 0, 0, 0 }, { 7, 11, 0 }, { 0, 8, 14 }, {
				0, 10, 11 }, { 0, 9, 10 }, { 0, 9, 9 }, { 0, 8, 15 },
		{ 7, 10, 0 }, { 0, 8, 12 }, { 0, 8, 11 }, { 0, 9, 13 }, { 0, 8, 8 }, {
				2, 11, 0 }, { 7, 14, 0 }, { 7, 13, 0 }, { 0, 8, 9 },
		{ 3, 15, 0 }, { 4, 10, 0 }, { 4, 15, 0 }, { 5, 9, 0 }, { 6, 8, 0 }, { 5,
				12, 0 }, { 5, 14, 0 }, { 6, 12, 0 }, { 6, 11, 0 }, { 6, 14, 0 },
		{ 0, 8, 10 }, { 0, 8, 13 }, { 0, 10, 8 }, { 7, 15, 0 }, { 0, 9, 15 }, {
				7, 12, 0 }, { 0, 9, 8 }, { 3, 9, 0 }, { 4, 14, 0 },
		{ 3, 12, 0 }, { 3, 14, 0 }, { 3, 8, 0 }, { 4, 12, 0 }, { 5, 8, 0 }, { 5,
				10, 0 }, { 3, 10, 0 }, { 7, 8, 0 }, { 6, 10, 0 }, { 4, 11, 0 },
		{ 4, 8, 0 }, { 4, 13, 0 }, { 3, 11, 0 }, { 4, 9, 0 }, { 6, 15, 0 }, { 3,
				13, 0 }, { 2, 15, 0 }, { 2, 14, 0 }, { 5, 11, 0 }, { 6, 13, 0 },
		{ 5, 13, 0 }, { 5, 15, 0 }, { 6, 9, 0 }, { 7, 9, 0 }, { 0, 10, 14 }, {
				0, 10, 9 }, { 0, 10, 15 }, { 0, 10, 10 }, { 0, 9, 12 }, { 0, 9,
				11 }, { 4, 0, 0 }, { 1, 11, 0 }, { 0, 12, 0 }, { 0, 11, 0 }, {
				1, 0, 0 }, { 0, 15, 0 }, { 1, 9, 0 }, { 0, 10, 0 }, { 5, 0, 0 },
		{ 2, 10, 0 }, { 1, 14, 0 }, { 0, 9, 0 }, { 0, 14, 0 }, { 6, 0, 0 }, { 3,
				0, 0 }, { 1, 8, 0 }, { 2, 8, 0 }, { 7, 0, 0 }, { 0, 8, 0 }, { 2,
				0, 0 }, { 0, 13, 0 }, { 1, 13, 0 }, { 1, 12, 0 }, { 1, 15, 0 },
		{ 1, 10, 0 }, { 2, 9, 0 }, { 0, 10, 12 }, { 0, 9, 14 }, { 0, 10, 12 }, {
				0, 11, 8 }, { 2, 10, 15 }, { 2, 11, 8 }, { 2, 11, 9 }, { 2, 11,
				10 }, { 2, 11, 11 }, { 2, 11, 12 }, { 2, 11, 13 },
		{ 2, 11, 14 }, { 2, 11, 15 }, { 2, 12, 8 }, { 2, 12, 9 }, { 2, 12, 10 },
		{ 2, 12, 11 }, { 2, 12, 12 }, { 2, 12, 13 }, { 2, 12, 14 },
		{ 2, 12, 15 }, { 2, 13, 8 }, { 2, 13, 9 }, { 2, 13, 10 }, { 2, 13, 11 },
		{ 2, 13, 12 }, { 2, 13, 13 }, { 2, 13, 14 }, { 2, 13, 15 },
		{ 2, 14, 8 }, { 2, 14, 9 }, { 2, 14, 10 }, { 2, 14, 11 }, { 2, 14, 12 },
		{ 2, 14, 13 }, { 2, 14, 14 }, { 2, 14, 15 }, { 0, 11, 9 },
		{ 0, 11, 10 }, { 0, 11, 11 }, { 0, 11, 12 }, { 0, 11, 13 },
		{ 0, 11, 14 }, { 0, 11, 15 }, { 0, 12, 8 }, { 0, 12, 9 }, { 0, 12, 10 },
		{ 0, 12, 11 }, { 0, 12, 12 }, { 0, 12, 13 }, { 0, 12, 14 },
		{ 0, 12, 15 }, { 0, 13, 8 }, { 0, 13, 9 }, { 0, 13, 10 }, { 0, 13, 11 },
		{ 0, 13, 12 }, { 0, 13, 13 }, { 0, 13, 14 }, { 0, 13, 15 },
		{ 0, 14, 8 }, { 0, 14, 9 }, { 0, 14, 10 }, { 0, 14, 11 }, { 0, 14, 12 },
		{ 0, 14, 13 }, { 0, 14, 14 }, { 0, 14, 15 }, { 0, 15, 8 }, { 0, 15, 9 },
		{ 0, 15, 10 }, { 0, 15, 11 }, { 0, 15, 12 }, { 0, 15, 13 },
		{ 0, 15, 14 }, { 0, 15, 15 }, { 1, 8, 8 }, { 1, 8, 9 }, { 1, 8, 10 }, {
				1, 8, 11 }, { 1, 8, 12 }, { 1, 8, 13 }, { 1, 8, 14 },
		{ 1, 8, 15 }, { 1, 9, 8 }, { 1, 9, 9 }, { 1, 9, 10 }, { 1, 9, 11 }, { 1,
				9, 12 }, { 1, 9, 13 }, { 1, 9, 14 }, { 1, 9, 15 }, { 1, 10, 8 },
		{ 1, 10, 9 }, { 1, 10, 10 }, { 1, 10, 11 }, { 1, 10, 12 },
		{ 1, 10, 13 }, { 1, 10, 14 }, { 1, 10, 15 }, { 1, 11, 8 }, { 1, 11, 9 },
		{ 1, 11, 10 }, { 1, 11, 11 }, { 1, 11, 12 }, { 1, 11, 13 },
		{ 1, 11, 14 }, { 1, 11, 15 }, { 1, 12, 8 }, { 1, 12, 9 }, { 1, 12, 10 },
		{ 1, 12, 11 }, { 1, 12, 12 }, { 1, 12, 13 }, { 1, 12, 14 },
		{ 1, 12, 15 }, { 1, 13, 8 }, { 1, 13, 9 }, { 1, 13, 10 }, { 1, 13, 11 },
		{ 1, 13, 12 }, { 1, 13, 13 }, { 1, 13, 14 }, { 1, 13, 15 },
		{ 1, 14, 8 }, { 1, 14, 9 }, { 1, 14, 10 }, { 1, 14, 11 }, { 1, 14, 12 },
		{ 1, 14, 13 }, { 1, 14, 14 }, { 1, 14, 15 }, { 1, 15, 8 },

		/* Secondary alphabet */
		{ 6, 15, 9 }, { 6, 15, 10 }, { 6, 15, 11 }, { 6, 15, 12 },
		{ 6, 15, 13 }, { 6, 15, 14 }, { 6, 15, 15 }, { 7, 8, 8 }, { 4, 10, 12 },
		{ 7, 8, 9 }, { 7, 8, 10 }, { 7, 8, 11 }, { 7, 8, 12 }, { 4, 10, 13 }, {
				7, 8, 13 }, { 7, 8, 14 }, { 7, 8, 15 }, { 7, 9, 8 },
		{ 7, 9, 9 }, { 7, 9, 10 }, { 7, 9, 11 }, { 7, 9, 12 }, { 7, 9, 13 }, {
				7, 9, 14 }, { 7, 9, 15 }, { 7, 10, 8 }, { 7, 10, 9 }, { 7, 10,
				10 }, { 7, 10, 11 }, { 7, 10, 12 }, { 7, 10, 13 },
		{ 7, 10, 14 }, { 3, 8, 8 }, { 4, 15, 11 }, { 5, 8, 14 }, { 5, 10, 11 },
		{ 5, 9, 10 }, { 5, 9, 9 }, { 5, 8, 15 }, { 4, 15, 10 }, { 5, 8, 12 }, {
				5, 8, 11 }, { 5, 9, 13 }, { 5, 8, 8 }, { 4, 10, 11 }, { 4, 15,
				14 }, { 4, 15, 13 }, { 5, 8, 9 }, { 4, 11, 15 }, { 4, 12, 10 },
		{ 4, 12, 15 }, { 4, 13, 9 }, { 4, 14, 8 }, { 4, 13, 12 }, { 4, 13, 14 },
		{ 4, 14, 12 }, { 4, 14, 11 }, { 4, 14, 14 }, { 5, 8, 10 }, { 5, 8, 13 },
		{ 5, 10, 8 }, { 4, 15, 15 }, { 5, 9, 15 }, { 4, 15, 12 }, { 5, 9, 8 }, {
				4, 11, 9 }, { 4, 12, 14 }, { 4, 11, 12 }, { 4, 11, 14 }, { 4,
				11, 8 }, { 4, 12, 12 }, { 4, 13, 8 }, { 4, 13, 10 },
		{ 4, 11, 10 }, { 4, 15, 8 }, { 4, 14, 10 }, { 4, 12, 11 }, { 4, 12, 8 },
		{ 4, 12, 13 }, { 4, 11, 11 }, { 4, 12, 9 }, { 4, 14, 15 },
		{ 4, 11, 13 }, { 4, 10, 15 }, { 4, 10, 14 }, { 4, 13, 11 },
		{ 4, 14, 13 }, { 4, 13, 13 }, { 4, 13, 15 }, { 4, 14, 9 }, { 4, 15, 9 },
		{ 5, 10, 14 }, { 5, 10, 9 }, { 5, 10, 15 }, { 5, 10, 10 }, { 5, 9, 12 },
		{ 5, 9, 11 }, { 3, 8, 12 }, { 4, 9, 11 }, { 4, 8, 12 }, { 4, 8, 11 }, {
				3, 8, 9 }, { 4, 8, 15 }, { 4, 9, 9 }, { 4, 8, 10 },
		{ 3, 8, 13 }, { 4, 10, 10 }, { 4, 9, 14 }, { 4, 8, 9 }, { 4, 8, 14 }, {
				3, 8, 14 }, { 3, 8, 11 }, { 4, 9, 8 }, { 4, 10, 8 },
		{ 3, 8, 15 }, { 4, 8, 8 }, { 3, 8, 10 }, { 4, 8, 13 }, { 4, 9, 13 }, {
				4, 9, 12 }, { 4, 9, 15 }, { 4, 9, 10 }, { 4, 10, 9 }, { 5, 10,
				12 }, { 5, 9, 14 }, { 5, 10, 12 }, { 5, 11, 8 }, { 7, 10, 15 },
		{ 7, 11, 8 }, { 7, 11, 9 }, { 7, 11, 10 }, { 7, 11, 11 }, { 7, 11, 12 },
		{ 7, 11, 13 }, { 7, 11, 14 }, { 7, 11, 15 }, { 7, 12, 8 }, { 7, 12, 9 },
		{ 7, 12, 10 }, { 7, 12, 11 }, { 7, 12, 12 }, { 7, 12, 13 },
		{ 7, 12, 14 }, { 7, 12, 15 }, { 7, 13, 8 }, { 7, 13, 9 }, { 7, 13, 10 },
		{ 7, 13, 11 }, { 7, 13, 12 }, { 7, 13, 13 }, { 7, 13, 14 },
		{ 7, 13, 15 }, { 7, 14, 8 }, { 7, 14, 9 }, { 7, 14, 10 }, { 7, 14, 11 },
		{ 7, 14, 12 }, { 7, 14, 13 }, { 7, 14, 14 }, { 7, 14, 15 },
		{ 5, 11, 9 }, { 5, 11, 10 }, { 5, 11, 11 }, { 5, 11, 12 },
		{ 5, 11, 13 }, { 5, 11, 14 }, { 5, 11, 15 }, { 5, 12, 8 }, { 5, 12, 9 },
		{ 5, 12, 10 }, { 5, 12, 11 }, { 5, 12, 12 }, { 5, 12, 13 },
		{ 5, 12, 14 }, { 5, 12, 15 }, { 5, 13, 8 }, { 5, 13, 9 }, { 5, 13, 10 },
		{ 5, 13, 11 }, { 5, 13, 12 }, { 5, 13, 13 }, { 5, 13, 14 },
		{ 5, 13, 15 }, { 5, 14, 8 }, { 5, 14, 9 }, { 5, 14, 10 }, { 5, 14, 11 },
		{ 5, 14, 12 }, { 5, 14, 13 }, { 5, 14, 14 }, { 5, 14, 15 },
		{ 5, 15, 8 }, { 5, 15, 9 }, { 5, 15, 10 }, { 5, 15, 11 }, { 5, 15, 12 },
		{ 5, 15, 13 }, { 5, 15, 14 }, { 5, 15, 15 }, { 6, 8, 8 }, { 6, 8, 9 }, {
				6, 8, 10 }, { 6, 8, 11 }, { 6, 8, 12 }, { 6, 8, 13 },
		{ 6, 8, 14 }, { 6, 8, 15 }, { 6, 9, 8 }, { 6, 9, 9 }, { 6, 9, 10 }, { 6,
				9, 11 }, { 6, 9, 12 }, { 6, 9, 13 }, { 6, 9, 14 }, { 6, 9, 15 },
		{ 6, 10, 8 }, { 6, 10, 9 }, { 6, 10, 10 }, { 6, 10, 11 }, { 6, 10, 12 },
		{ 6, 10, 13 }, { 6, 10, 14 }, { 6, 10, 15 }, { 6, 11, 8 }, { 6, 11, 9 },
		{ 6, 11, 10 }, { 6, 11, 11 }, { 6, 11, 12 }, { 6, 11, 13 },
		{ 6, 11, 14 }, { 6, 11, 15 }, { 6, 12, 8 }, { 6, 12, 9 }, { 6, 12, 10 },
		{ 6, 12, 11 }, { 6, 12, 12 }, { 6, 12, 13 }, { 6, 12, 14 },
		{ 6, 12, 15 }, { 6, 13, 8 }, { 6, 13, 9 }, { 6, 13, 10 }, { 6, 13, 11 },
		{ 6, 13, 12 }, { 6, 13, 13 }, { 6, 13, 14 }, { 6, 13, 15 },
		{ 6, 14, 8 }, { 6, 14, 9 }, { 6, 14, 10 }, { 6, 14, 11 }, { 6, 14, 12 },
		{ 6, 14, 13 }, { 6, 14, 14 }, { 6, 14, 15 }, { 6, 15, 8 }, };

// Blantantly taken from WiringPi
void delayMicrosecondsHard(unsigned int howLong) {
	struct timeval tNow, tLong, tEnd;

	gettimeofday(&tNow, NULL);
	tLong.tv_sec = howLong / 1000000;
	tLong.tv_usec = howLong % 1000000;
	timeradd (&tNow, &tLong, &tEnd)
	;

	while (timercmp (&tNow, &tEnd, <))
		gettimeofday(&tNow, NULL);
}
// Read from the system Timer (See BCM2835 Library)

void delayMicrosecs(unsigned int howLong) {
	struct timespec sleeper;
	unsigned int uSecs = howLong % 1000000;
	unsigned int wSecs = howLong / 1000000;

	/**/if (howLong == 0)
		return;
	else if (howLong < 100)
		delayMicrosecondsHard(howLong);
	else {
		sleeper.tv_sec = wSecs;
		sleeper.tv_nsec = (long) (uSecs * 1000L);
		nanosleep(&sleeper, NULL);
	}
}

void domex_tone_bb(int tone) {
	// BitBang
	// This get's us close but not quite close enough.
	int i = 0;
	int spacer;
	if (tone > 0) {
		spacer = 4224 / (tone * 10);
	} else {
		spacer = 42240;
	}
	for (i = 0; i < 42240; i++) {// 4224 should be the magic number for domex16 tones. (This is 10x the pwm rate for
		if ((i % spacer) == 0) {
			GPIO_SET = 1 << 18;
		} else {
			GPIO_CLR = 1 << 18;
		}
		delayMicrosecs(1); // Ideally we need to delay for ~1.5 uS - Not easily possible though :(
	}
}
void domex_tone(int tone) {
	// Total length of this function should be 1/15.625s
	//printf("DomEX: Sending tone %d\n", tone);
	//domex_tone_bb(tone);	// Used for Bit Bang Method

	// 32 bits = 2 milliseconds, init with 1 millisecond
	//setServo(0);

	// PWM
	// Set tone
	// delayMicroseconds(64000);
	*(pwm + PWM0_DATA) = tone + 100;
	delayMicrosecs(64000); // 64mS per tone for 15.625 baud
}

void domex_nibble(int nibble) {
	static int currtone = 0;
	currtone += nibble;
	currtone += 2;
	if (currtone >= 18) {
		currtone -= 18;
	}
	domex_tone(currtone);
}

void domex_txchar(uint16_t code) {
	uint8_t i, c;

	for (i = 0; i < 3; i++) {
		c = varicode[code][i];
		if (i && !(c & 0x8))
			break;
		domex_nibble(c);
	}
}

void domex_txstring(char *s) {
	GPIO_SET = 1 << 23;	// Enable NTX2b
	for (; *s; s++)
		domex_txchar(*s);
	GPIO_CLR = 1 << 23;	// Disable NTX2b
}

void sig_handler(int sig) {
	printf("Got Signal %d\n", sig);
	switch (sig) {
	case SIGINT:
		// Interupt (Ctrl c From command line) - Graceful shutdown
		printf("Sig: Got SIGINT - Shutting down\n");
		//systemloop=false;
		GPIO_CLR = 1 << 23;
		break;
	case SIGALRM:
		printf("Sig: Got SIGALRM - Next Tone\n");
		break;
	}
}

void SetupPWM() {
	uint32_t pwm_control;

//SetMode (Balanced)
	// start PWM0
	*(pwm + PWM_CONTROL) = PWM0_ENABLE;			// Balanced mode
	//*(pwm + PWM_CONTROL) = PWM0_MS_MODE|PWM0_ENABLE;	// Mark Space mode (High for Data, then Low until Range)
	//*(pwm + PWM_CONTROL) = PWM0_SERIAL|PWM0_ENABLE;	// Serializer mode

// setRange (1024)
	// filled with 0 for 20 milliseconds = 320 bits
	*(pwm + PWM0_RANGE) = 384;	// 1024 in WiringPi
	delayMicrosecs(10);

//setClock(32)	- 19.2 /32 = 600khz
	// Preserve Current Control
	pwm_control = *(pwm + PWM_CONTROL);

	// Stop PWM Control (Needed for MS Mode)
	*(pwm + PWM_CONTROL) = 0;

	// stop clock and waiting for busy flag doesn't work, so kill clock
	*(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x01;		//(1 << 5);
	delayMicrosecs(110);

	// Wait for clock to be !BUSY
	while ((*(clk + PWMCLK_CNTL) & 0x80) != 0)
		delayMicrosecs(1);

	// set frequency
	// DIVI is the integer part of the divisor
	// the fractional part (DIVF) drops clock cycles to get the output frequency, bad for servo motors
	// 320 bits for one cycle of 20 milliseconds = 62.5 us per bit = 16 kHz
	int idiv = 5;	//(int) (19200000.0f / 16000.0f);
	if (idiv < 1 || idiv > 0x1000) {
		printf("idiv out of range: %x\n", idiv);
		exit(-1);
	}
	*(clk + PWMCLK_DIV) = 0x5A000000 | (idiv << 12);

	// Start PWM Clock
	*(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x11;

	// Restore PWM Settings
	*(pwm + PWM_CONTROL) = pwm_control;

}

void domex_setup(void) {

	//int rep;

	signal(SIGINT, sig_handler);
	signal(SIGALRM, sig_handler);

	// Set up gpi pointer for direct register access
	setup_io();

	// GPIO23 - NTX2 enable
	INP_GPIO(23); // must use INP_GPIO before we can use OUT_GPIO
	OUT_GPIO(23);

	// GPIO40 and GPIO45 are audio and also support PWM	Connect to analogue audio circitury via R21 and R27
	// GPIO18 - NTX2 Data
	// Set GPIO18 to PWM Function
	INP_GPIO(18); // must use INP_GPIO before we can use OUT_GPIO or SET_GPIO_ALT

	//OUT_GPIO(18);	// For BitBang
	SET_GPIO_ALT(18, 5);	// For PWM
	delayMicrosecs(110);
	SetupPWM();

	/*
	 // Setup timer
	 struct itimerval new,old;

	 new.it_interval.tv_usec = 0;
	 new.it_interval.tv_sec = 0;
	 new.it_value.tv_usec = 10;
	 new.it_value.tv_sec = 0;
	 setitimer (ITIMER_REAL, &new, &old);
	 */

}