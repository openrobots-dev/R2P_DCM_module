/*
 ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
 2011 Giovanni Di Sirio.

 This file is part of ChibiOS/RT.

 ChibiOS/RT is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 ChibiOS/RT is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "halconf.h"
#include "test.h"
#include "shell.h"
#include "chprintf.h"

#define WA_SIZE_1K      THD_WA_SIZE(1024)

int16_t pwm = 0;
uint16_t qei = 0;

adcsample_t vcs_buffer[128];
adcsample_t vcs_mean;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(4096)
#define TEST_WA_SIZE    THD_WA_SIZE(1024)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
	size_t n, size;

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: mem\r\n");
		return;
	}
	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
	static const char *states[] = { THD_STATE_NAMES };
	Thread *tp;

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: threads\r\n");
		return;
	}
	chprintf(chp, "    addr    stack prio refs     state time\r\n");
	tp = chRegFirstThread();
	do {
		chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n", (uint32_t) tp,
				(uint32_t) tp->p_ctx.r13, (uint32_t) tp->p_prio,
				(uint32_t)(tp->p_refs - 1), states[tp->p_state],
				(uint32_t) tp->p_time);
		tp = chRegNextThread(tp);
	} while (tp != NULL);
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
	Thread *tp;

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: test\r\n");
		return;
	}
	tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriority(), TestThread,
			chp);
	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}
	chThdWait(tp);
}

static void cmd_pwm(BaseSequentialStream *chp, int argc, char *argv[]) {

	if (argc != 1) {
		chprintf(chp, "Usage: pwm <value>\r\n");
		return;
	}
	pwm = (int16_t) atoi(argv[0]);
}

static void cmd_en(BaseSequentialStream *chp, int argc, char *argv[]) {

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: en\r\n");
		return;
	}

	palSetPad(DRIVER_GPIO, DRIVER_RESET);
}
static void cmd_dis(BaseSequentialStream *chp, int argc, char *argv[]) {

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: dis\r\n");
		return;
	}

	palClearPad(DRIVER_GPIO, DRIVER_RESET);
}

static const ShellCommand commands[] = { { "mem", cmd_mem }, { "threads",
		cmd_threads }, { "test", cmd_test }, { "pwm", cmd_pwm },
		{ "en", cmd_en }, { "dis", cmd_dis }, { NULL, NULL } };

static const ShellConfig shell_cfg1 = { (BaseSequentialStream *) &SERIAL_DRIVER,
		commands };

/*===========================================================================*/
/* PWM related.                                                        */
/*===========================================================================*/

/*
 * PWM cyclic callback.
 */
static void pwmcb(PWMDriver *pwmp) {

	(void) pwmp;
	chSysLockFromIsr()
	;
	if (pwm >= 0) {
		pwm_lld_enable_channel(&PWM_DRIVER, 0, pwm);
		pwm_lld_enable_channel(&PWM_DRIVER, 1, 0);
	} else {
		pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);
		pwm_lld_enable_channel(&PWM_DRIVER, 1, -pwm);
	}
	chSysUnlockFromIsr();
}

/*
 * PWM configuration.
 */
static PWMConfig pwmcfg = { 72000000, /* 72MHz PWM clock frequency.   */
4096, /* 12-bit PWM, 17KHz frequency. */
pwmcb, {
		{ PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL },
		{ PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL },
		{ PWM_OUTPUT_DISABLED, NULL }, { PWM_OUTPUT_DISABLED, NULL } }, 0,
#if STM32_PWM_USE_ADVANCED
		72, /* XXX 1uS deadtime insertion   */
#endif
		};

/*===========================================================================*/
/* ADC related.                                                              */
/*===========================================================================*/

/*
 * ADC streaming callback.
 */
size_t nx = 0, ny = 0;
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
	uint32_t tmp = 0;

	(void) adcp;
	while (n-- > 0) {
		tmp += *(buffer++);
	}

	vcs_mean = (adcsample_t) (tmp >> 6);
}

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN10.
 */
static const ADCConversionGroup adcgrpcfg1 = { TRUE, 1, adccallback, NULL, 0, 0, /* CR1, CR2 */
0, ADC_SMPR2_SMP_AN3(ADC_SAMPLE_71P5), ADC_SQR1_NUM_CH(1), 0, /* SQR2 */
ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) };

/*===========================================================================*/
/* QEI related.                                                              */
/*===========================================================================*/

static QEIConfig qeicfg = {
	QEI_MODE_QUADRATURE,
	QEI_BOTH_EDGES,
	QEI_DIRINV_FALSE,
};

/*===========================================================================*/
/* CAN related.                                                              */
/*===========================================================================*/

/*
 * CAN configuration.
 */
static const CANConfig can_cfg = { NULL, NULL, NULL, CAN_MCR_NART,
		CAN_BTR_SJW(0) | CAN_BTR_TS2(2) | CAN_BTR_TS1(4) | CAN_BTR_BRP(3) };

/*===========================================================================*/
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

	(void) arg;

	chRegSetThreadName("blinker");
	while (TRUE) {
		palTogglePad(LED_GPIO, LED1);
		chThdSleepMilliseconds(500);
	}
	return 0;
}

/*
 * Application entry point.
 */
int main(void) {
	Thread *shelltp = NULL;

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	/*
	 * Activates the serial driver 1 using the driver default configuration.
	 */
	sdStart(&SERIAL_DRIVER, NULL);

	/*
	 * Shell manager initialization.
	 */
	shellInit();

	/*
	 * Activates the ADC1 driver.
	 */
	adcStart(&ADCD1, NULL);
	adcStartConversion(&ADCD1, &adcgrpcfg1, vcs_buffer, 128);

	/*
	 * Activates the QEI driver.
	 */
	qeiStart(&QEI_DRIVER, &qeicfg);
	qeiEnable(&QEI_DRIVER);

	/*
	 * Activates the PWM driver.
	 */
	palSetPad(DRIVER_GPIO, DRIVER_RESET);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	/*
	 * Activates the CAN1 driver.
	 */
	canStart(&CAND1, &can_cfg);

	/*
	 * Creates the blinker thread.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (TRUE) {
		if (!shelltp)
			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(shelltp)) {
			chThdRelease(shelltp);
			shelltp = NULL;
		}
		qei = qeiGetCount(&QEI_DRIVER);
		chprintf((BaseSequentialStream *) &SERIAL_DRIVER, "PWM: %4d QEI: %4d VCS: %4d\r\n",
				pwm, qei, vcs_mean);
		chThdSleepMilliseconds(200);
	}
}
