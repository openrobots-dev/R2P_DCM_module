#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"

typedef enum {
	UNINIT = 0,
	PWM = 1,
	CURRENT = 2,
	SPEED = 3,
} control_t;

/* Control policy. */
control_t control = UNINIT;


/*===========================================================================*/
/* PWM related.                                                              */
/*===========================================================================*/

/* PWM value. */
int16_t pwm = 0;

/*
 * PWM callback.
 */
static void pwmcb(PWMDriver *pwmp) {
	chSysLockFromIsr();

	if (pwm > 0) {
		pwm_lld_enable_channel(pwmp, 0, 0);
		pwm_lld_enable_channel(pwmp, 1, pwm);
	} else {
		pwm_lld_enable_channel(pwmp, 0, -pwm);
		pwm_lld_enable_channel(pwmp, 1, 0);
	}
	chSysUnlockFromIsr();
}

/*
 * PWM configuration.
 */
PWMConfig pwmcfg = { STM32_SYSCLK, /* 72MHz PWM clock frequency.   */
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
/* Quadrature encoder related.                                               */
/*===========================================================================*/
/*
 * QEI configuration.
 */
QEIConfig qeicfg = {
        QEI_MODE_QUADRATURE,
        QEI_BOTH_EDGES,
        QEI_DIRINV_FALSE,
};

/*===========================================================================*/
/* Current sense related.                                                    */
/*===========================================================================*/
#define ADC_NUM_CHANNELS   1
#define ADC_BUF_DEPTH      8

adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

/*
 * ADC conversion group.
 * Mode:        Circular buffer, 8 samples of 1 channel.
 * Channels:    IN10.
 */
static const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_NUM_CHANNELS,
  NULL,
  NULL,
  0, 0,                         /* CR1, CR2 */
  0,
  ADC_SMPR2_SMP_AN3(ADC_SAMPLE_239P5), /* SMPR2 */
  ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS),
  0,                            /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3)
};


static const ADCConversionGroup adcgrpcfg2 = {
  FALSE,
  ADC_NUM_CHANNELS,
  NULL,
  NULL,
  0, 0,                         /* CR1, CR2 */
  0,
  ADC_SMPR2_SMP_AN3(ADC_SAMPLE_239P5), /* SMPR2 */
  ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS),
  0,                            /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3)
};

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(4096)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
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
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state time\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
            states[tp->p_state], (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_pwm(BaseSequentialStream *chp, int argc, char *argv[]) {

  if (argc != 1) {
    chprintf(chp, "Usage: p <setpoint>\r\n");
    return;
  }

  control = PWM;
  pwm = atoi(argv[0]);
}

static void cmd_current(BaseSequentialStream *chp, int argc, char *argv[]) {

  if (argc != 1) {
    chprintf(chp, "Usage: c <setpoint>\r\n");
    return;
  }

  (void) argv;

  control = CURRENT;
}

static void cmd_speed(BaseSequentialStream *chp, int argc, char *argv[]) {

  if (argc != 1) {
    chprintf(chp, "Usage: s <setpoint>\r\n");
    return;
  }

  (void) argv;

  control = SPEED;
}

static void cmd_dump(BaseSequentialStream *chp, int argc, char *argv[]) {

  if (argc != 0) {
    chprintf(chp, "Usage: d\r\n");
    return;
  }

  (void) argv;

  chprintf(chp, "p: %d cs: %d\r\n", pwm, adc_samples[0]);
}

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"p", cmd_pwm},
  {"c", cmd_current},
  {"s", cmd_speed},
  {"d", cmd_dump},
  {NULL, NULL}
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&SERIAL_DRIVER,
  commands
};

/*===========================================================================*/
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Motor control thread.
 */
msg_t control_thread(void * arg) {
	int16_t step = 100;

	(void) arg;

	chRegSetThreadName("control");

	for (;;) {
		pwm += step;

		if ((pwm >= 1000) || (pwm <= -1000)) {
			step = -step;
		}

		chThdSleepMilliseconds(500);
	}

	return CH_SUCCESS;
}

/*
 * Application entry point.
 */
int main(void) {
	Thread *shelltp = NULL;

	halInit();
	chSysInit();

	/* Shell manager initialization. */
	shellInit();

	/* Start the serial driver. */
	sdStart(&SERIAL_DRIVER, NULL);

	/* Reset the h-bridge driver IC. */
	palSetPad(DRIVER_GPIO, DRIVER_RESET);
	chThdSleepMilliseconds(500);

	/* Start the PWM driver. */
	pwmStart(&PWM_DRIVER, &pwmcfg);

	/* Set motor to safe state. */
	pwmEnableChannel(&PWM_DRIVER, 0, 0);
	pwmEnableChannel(&PWM_DRIVER, 1, 0);

	/* Start the QEI driver. */
	qeiStart(&QEI_DRIVER, &qeicfg);

	/* Start the ADC driver. */
	adcStart(&ADC_DRIVER, NULL);

    /* Starts an ADC continuous conversion. */
	adcStartConversion(&ADC_DRIVER, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);

	control = PWM;

	/* Spawn the control thread. */
	chThdCreateFromHeap(NULL, THD_WA_SIZE(1024), NORMALPRIO, control_thread, NULL);

	for (;;) {
		if (!shelltp)
		  shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(shelltp)) {
		  chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
		  shelltp = NULL;           /* Triggers spawning of a new shell.        */
		}

		palTogglePad(LED1_GPIO, LED1);
		chThdSleepMilliseconds(500);
	}

	return CH_SUCCESS;
}
