#include "ch.h"
#include "hal.h"

#include "rtcan.h"
#include "config.h"

#include <r2p/common.hpp>
#include <r2p/Middleware.hpp>
#include <r2p/Node.hpp>
#include <r2p/Topic.hpp>
#include <r2p/Publisher.hpp>
#include <r2p/Subscriber.hpp>
#include <r2p/Mutex.hpp>
#include <r2p/NamingTraits.hpp>
#include <r2p/Bootloader.hpp>
#include "r2p/transport/RTCANTransport.hpp"

#include "r2p/msg/motor.hpp"
#include "r2p/node/led.hpp"
#include "r2p/node/motor.hpp"
#include "r2p/msg/std_msgs.hpp"
#include "r2p/node/pid.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "R2PMODX"
#endif

extern "C" {
void *__dso_handle;
void __cxa_pure_virtual() {
	chSysHalt();
}
void _exit(int) {
	chSysHalt();
	for (;;) {
	}
}
int _kill(int, int) {
	chSysHalt();
	return -1;
}
int _getpid() {
	return 1;
}
} // extern "C"

static WORKING_AREA(wa_info, 2048);

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME,
		"BOOT_"R2P_MODULE_NAME);

// RTCAN transport
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

/*
 * Steer node.
 */

#define _TICKS 2000.0f
#define _RATIO 73.0f * (85.0f / 32.0f) * 10.0 // 10.0 is steerwheel to wheel angle
//#define _RATIO 14.0f
#define _PI 3.14159265359f

#define R2T(r) ((r / (2 * _PI)) * (_TICKS * _RATIO))
#define T2R(t) ((t / (_TICKS * _RATIO)) * (2 * _PI))

extern PWMConfig pwmcfg;
extern QEIConfig qeicfg;

PID steer_pid;
float steer_position = 0;
r2p::Time last_setpoint(0);

bool steer_callback(const r2p::Velocity3Msg &msg) {

	if (fabs(msg.w) <= 0.7) {
		steer_pid.set(msg.w);
	}

	last_setpoint = r2p::Time::now();
	palTogglePad(LED2_GPIO, LED2); palSetPad(LED4_GPIO, LED4);

	return true;
}

msg_t steer_node(void * arg) {
	r2p::Node node("pid");
	r2p::Subscriber<r2p::Velocity3Msg, 2> vel_sub(steer_callback);
	r2p::Publisher<r2p::EncoderMsg> steer_pub;
	r2p::EncoderMsg * msgp;

	(void) arg;

	chRegSetThreadName("steer_node");

	/* Start PWM driver. */
	palSetPad(DRIVER_GPIO, DRIVER_RESET);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	/* Start QEI driver. */
	qeiStart(&QEI_DRIVER, &qeicfg);
	qeiEnable (&QEI_DRIVER);

	steer_pid.config(20000.0, 5.0, 0.0, 0.01, -4095.0, 4095.0);
	node.subscribe(vel_sub, "vel_cmd");
	node.advertise(steer_pub, "steer_encoder");

	chThdSleepMilliseconds(100);

	for (;;) {
		node.spin(TIME_IMMEDIATE);

		systime_t time = chTimeNow();

		if (r2p::Time::now() - last_setpoint > r2p::Time::ms(500)) {
			steer_pid.set(0);
			palTogglePad(LED4_GPIO, LED4);
		}

		steer_position += T2R(qeiUpdate(&QEI_DRIVER));
		int pwm = steer_pid.update(steer_position);

		chSysLock()
		;
		if (pwm > 0) {
			pwm_lld_enable_channel(&PWM_DRIVER, 0, pwm);
			pwm_lld_enable_channel(&PWM_DRIVER, 1, 0);
		} else {
			pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);
			pwm_lld_enable_channel(&PWM_DRIVER, 1, -pwm);
		}
		chSysUnlock();

		if (steer_pub.alloc(msgp)) {
			msgp->delta = steer_position;
			steer_pub.publish(*msgp);
		}

		time += MS2ST(10);
		chThdSleepUntil(time);
	}

	return CH_SUCCESS;
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	palClearPad(LED1_GPIO, LED1); palClearPad(LED2_GPIO, LED2); palClearPad(LED3_GPIO, LED3); palClearPad(LED4_GPIO, LED4);
	chThdSleepMilliseconds(500); palSetPad(LED1_GPIO, LED1); palSetPad(LED2_GPIO, LED2); palSetPad(LED3_GPIO, LED3); palSetPad(LED4_GPIO, LED4);

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info),
			r2p::Thread::LOWEST);

	rtcantra.initialize(rtcan_config);

	r2p::Middleware::instance.start();

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO,
			r2p::ledsub_node, NULL);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(4096), NORMALPRIO + 1,
			steer_node, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}
	return CH_SUCCESS;
}
}
