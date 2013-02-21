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

#include "rtcan.h"
#include "Middleware.hpp"
#include "topics.h"
#include "uid.h"

#define WA_SIZE_1K      THD_WA_SIZE(1024)
#define WA_SIZE_2K      THD_WA_SIZE(2048)
#define WA_SIZE_4K      THD_WA_SIZE(4096)

extern RTCANDriver RTCAND;

int16_t pwm = 0;
uint16_t qei = 0;

adcsample_t vcs_buffer[128];
adcsample_t vcs_mean;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(4096)
#define TEST_WA_SIZE    THD_WA_SIZE(1024)
#define WA_SIZE_512B    THD_WA_SIZE(512)

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
static PWMConfig pwmcfg = { STM32_SYSCLK, /* 72MHz PWM clock frequency.   */
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
/* Middleware test related.                                                  */
/*===========================================================================*/

/*
 * Publisher threads.
 */

static msg_t QEIPublisherThread(void *arg) {
	Middleware & mw = Middleware::instance();
	Node n("qei_pub");
	Publisher<QEI> pub("qei");
	QEI *msg;
	systime_t time;

	(void) arg;
	chRegSetThreadName("QEI PUB THD");

	mw.newNode(&n);

	if (n.advertise(&pub)) {
		chprintf((BaseSequentialStream*)&SERIAL_DRIVER, "qei pub OK\r\n");
	} else {
		chprintf((BaseSequentialStream*)&SERIAL_DRIVER, "qei pub FAIL\r\n");
			mw.delNode(&n);
			while(1);
	}

	time = chTimeNow();

	while (TRUE) {
		time += MS2ST(10);
		msg = pub.alloc();
		if (msg != NULL) {
			chSysLock();
			msg->timestamp = chTimeNow();
			msg->value = qeiGetCount(&QEI_DRIVER);
			chSysUnlock();
			pub.broadcast(msg);
		}
		chThdSleepUntil(time);
	}

	return 0;
}


static msg_t PWMSubscriberThread(void *arg) {
	Middleware & mw = Middleware::instance();
	Node n("pwm_sub");
	Subscriber<PWM, 5> sub("pwm");
	PWM *msg;

	(void) arg;
	chRegSetThreadName("PWM SUB THD");

	mw.newNode(&n);

	if (n.subscribe(&sub)) {
		chprintf((BaseSequentialStream*)&SERIAL_DRIVER, "pwm sub OK\r\n");
	} else {
		chprintf((BaseSequentialStream*)&SERIAL_DRIVER, "pwm sub QUEUED\r\n");
	}

	while (TRUE) {
		n.spin();
		while ((msg = sub.get()) != NULL) {
			pwm = msg->pwm;
			sub.release(msg);
		}
	}

	return 0;
}

static msg_t PWM123SubscriberThread(void *arg) {
	Middleware & mw = Middleware::instance();
	Node n("pwm123_sub");
	Subscriber<PWM3, 5> sub("pwm123");
	PWM3 *msg;

	(void) arg;
	chRegSetThreadName("PWM123 SUB THD");

	mw.newNode(&n);

	if (n.subscribe(&sub)) {
		chprintf((BaseSequentialStream*)&SERIAL_DRIVER, "pwm123 sub OK\r\n");
	} else {
		chprintf((BaseSequentialStream*)&SERIAL_DRIVER, "pwm123 sub QUEUED\r\n");
	}

	while (TRUE) {
		n.spin();
		while ((msg = sub.get()) != NULL) {
			switch (uid8()) {
			case 70:
				pwm = msg->pwm1;
				break;
			case 85:
				pwm = msg->pwm2;
				break;
			case 40:
				pwm = msg->pwm3;
				break;
			}
			sub.release(msg);
		}
	}

	return 0;
}

/*===========================================================================*/
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 512);
static msg_t Thread1(void *arg) {
	Middleware & mw = Middleware::instance();
	Node n("blinker");
	Subscriber<Led, 5> sub("led1");
	Led * led;

	(void) arg;
	chRegSetThreadName("blinker");

	mw.newNode(&n);

	if (n.subscribe(&sub)) {
		chprintf((BaseSequentialStream*)&SERIAL_DRIVER, "led sub OK\r\n");
	} else {
		chprintf((BaseSequentialStream*)&SERIAL_DRIVER, "led sub QUEUED\r\n");
	}

	while (TRUE) {
		n.spin(MS2ST(500));

		led = sub.get();
		if (led) {
			if (led->set) {
				palSetPad(LED_GPIO, led->pin);
			} else {
				palClearPad(LED_GPIO, led->pin);
			}
			sub.release(led);
		}

		palTogglePad(LED_GPIO, LED1);
	}

	return 0;
}

/*
 * Control thread, times are in milliseconds.
 */
static const int16_t controlLoopLength=10; //duration of the control loop [msec]

static msg_t ControllerThread(void *arg) {
	Middleware & mw = Middleware::instance();
	Node n("controller");
	Subscriber<SpeedSetpoint3, 2> sub("speed123");
	Subscriber<PIDSetup, 2> pidsub("pidsetup");
	SpeedSetpoint3 * speed;
	PIDSetup * pidsetup;

	int curEncoder; // current encoder value
	int oldEncoder=0; //previous encoder value
	int16_t dEncoder; //variation of encoder value
	int dT; //delta time occurred since the previous reading
	float errorSpeed; //error in speed [encoder ticks/msec]
	float oldErrorSpeed=0; //previous error in speed [encoder ticks/msec]
	float dErrorSpeed; //delta error in speed [encoder ticks/msec]
	int16_t maxSpeed = 200; //max speed of the motor in [encoder ticks/msec]
	int16_t percentualTargetSpeed; //target speed as a percentage over 10000 of the maximum speed
	int16_t targetSpeed; // actual target speed in [encoder ticks/msec]
	int16_t integralError; // integral of the error since the setup has been given in [encoder ticks/msec]
	float kP=7.2; //proportional constant for controller
	float kD=0.5; //derivative constant for controller
	float kI=0.5; //integral constant for controller
	int16_t outPwm; //pwm output of the controller [0..4000] CONTROLLARE VALORE

	systime_t trialLapse= 6000; //This is the length of the trial [msec]
	systime_t trialEndTime= chTimeNow()+trialLapse; //This is the end time of the trial
	systime_t loopTime= chTimeNow(); //This is the control loop time
	systime_t curTime; //This is the time at which the encoder is read
	systime_t oldTime = -controlLoopLength; //This is time at which the encoder was read before
	      	  	  	  	  	  	  	  	  	//The initialization is need for the first step

	(void) arg;
	chRegSetThreadName("controller");
	mw.newNode(&n);

	n.subscribe(&sub);
	n.subscribe(&pidsub);

	// *** assigning values to variables for test, to be changed later
	percentualTargetSpeed = 0; // *** It should come from outside
	   	   	   	   	  // It is a percentage over 10000 of the max speed expressed in encoder ticks.

    targetSpeed = percentualTargetSpeed*maxSpeed/10000;
    integralError=0;

    // Enable PWM output.
    palSetPad(DRIVER_GPIO, DRIVER_RESET);

//	while (chTimeNow()<trialEndTime) {
	while (1) {
		loopTime += MS2ST(controlLoopLength);

		curTime=chTimeNow();
		curEncoder = qeiGetCount(&QEI_DRIVER);

		dT = curTime-oldTime;
		oldTime = curTime; //The encoder time is synchronized on the encoder reading

		dEncoder = curEncoder-oldEncoder;
		if (abs(dEncoder) > 32767) dEncoder=dEncoder+65535;
		oldEncoder = curEncoder;
		errorSpeed = targetSpeed - dEncoder/dT;
		dErrorSpeed = errorSpeed - oldErrorSpeed;
		oldErrorSpeed = errorSpeed;\
		integralError=+errorSpeed;
		//INTEGRAL SPEED

		outPwm = kP*errorSpeed + kD*dErrorSpeed/dT +kI*integralError; // control law

		pwm = outPwm;

		// Check for new speed setpoint
		while ((speed = sub.get()) != NULL) {
			switch (uid8()) {
			case 70:
				percentualTargetSpeed = speed->speed1;
				targetSpeed = percentualTargetSpeed*maxSpeed/10000;
				break;
			case 85:
				percentualTargetSpeed = speed->speed2;
				targetSpeed = percentualTargetSpeed*maxSpeed/10000;
				break;
			case 40:
				percentualTargetSpeed = speed->speed3;
				targetSpeed = percentualTargetSpeed*maxSpeed/10000;
				break;
			}
		}

		// Check for new controller constants
		while ((pidsetup = pidsub.get()) != NULL) {
			kP = pidsetup->kp / 1000.0;
			kI = pidsetup->ki / 1000.0;
			kD = pidsetup->kd / 1000.0;
		}

		chThdSleepUntil(loopTime);

/*
		chprintf((BaseSequentialStream *) &SERIAL_DRIVER, "Start: %4d Now: %4d PWM: %4d  dEnc: %4d dErrSpeed: %4d\r\n",
				curTime, loopTime, pwm, dEncoder, dErrorSpeed);

		loopTime = chTimeNow(); //The control loop time is synchronized on the control loop
							//This is useless if we trust chThdSleepUntil(time)
*/
	}
	pwm = 0; //stops the motor at the end of the loop
	return 0;
}

/*
 * Application entry point.
 */

RemotePublisher led1rpub("led1", sizeof(Led));
RemotePublisher pwm123rpub("pwm123", sizeof(PWM3));
RemotePublisher speed123rpub("speed123", sizeof(SpeedSetpoint3));
RemotePublisher pidrpub("pidsetup", sizeof(PIDSetup));

RemoteSubscriberT<QEI, 5> rsub("qei");

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
//	adcStart(&ADCD1, NULL);
//	adcStartConversion(&ADCD1, &adcgrpcfg1, vcs_buffer, 128);

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
	 * Activates the RTCAN driver.
	 */
	rtcanInit();
	rtcanStart (NULL);

	/*
	 * Subscribe to remote publishers - should be done automagically by the middleware.
	 */
	Middleware & mw = Middleware::instance();
	led1rpub.id(LED23_ID);
	pwm123rpub.id(PWM123_ID);
	speed123rpub.id(SPEED123_ID);
	pidrpub.id(PIDSETUP_ID);
	mw.advertise(&led1rpub);
	mw.advertise(&pwm123rpub);
	mw.advertise(&speed123rpub);
	mw.advertise(&pidrpub);

	/*
	 * Creates the blinker thread.
	 */
	chThdCreateFromHeap (NULL, WA_SIZE_512B, NORMALPRIO, Thread1, NULL);

	/*
	 * Creates the publishers threads.
	 */
	chThdCreateFromHeap (NULL, WA_SIZE_1K, NORMALPRIO + 1, QEIPublisherThread, NULL);

	/*
	 * Creates the subscriber threads.
	 */
	chThdCreateFromHeap (NULL, WA_SIZE_1K, NORMALPRIO + 1, PWMSubscriberThread, NULL);
	chThdCreateFromHeap (NULL, WA_SIZE_1K, NORMALPRIO + 1, PWM123SubscriberThread, NULL);

	chThdSleepMilliseconds(100);

	/*
	 * Publish to remote subscribers - should be done automagically by the middleware.
	 */
	LocalPublisher * pub;

	switch (uid8()) {
	case 70:
		rsub.id(QEI1_ID);
		break;
	case 85:
		rsub.id(QEI2_ID);
		break;
	case 40:
		rsub.id(QEI3_ID);
		break;
	}

	pub = mw.findLocalPublisher("qei");

	if (pub) {
		rsub.subscribe(pub);
	}

	/*
	 * Creates the controller thread.
	 */
	chThdCreateFromHeap (NULL, WA_SIZE_1K, NORMALPRIO + 1, ControllerThread, NULL);

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (TRUE) {
//		if (!shelltp)
//			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
//		else if (chThdTerminated(shelltp)) {
//			chThdRelease(shelltp);
//			shelltp = NULL;
//		}
//		qei = qeiGetCount(&QEI_DRIVER);
//		chprintf((BaseSequentialStream *) &SERIAL_DRIVER, "PWM: %4d QEI: %4d VCS: %4d\r\n",
//				pwm, qei, vcs_mean);
		chThdSleepMilliseconds(200);
	}
}
