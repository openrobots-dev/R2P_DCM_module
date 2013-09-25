#include "ch.h"
#include "hal.h"

#include "rtcan.h"

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

#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "R2PMODX"
#endif

struct TiltMsg: public r2p::Message {
	float angle;
	float rate;
}R2P_PACKED;

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

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

// RTCAN transport
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

/*
 * Balance node.
 */
msg_t balance_node(void *) {
	TiltMsg sub_msgbuf[5], *sub_queue[5];
	r2p::Node node("balance");
	r2p::Subscriber<TiltMsg> sub(sub_queue, 5);
	TiltMsg * msgp;

	node.subscribe(sub, "tilt", sub_msgbuf);

	for (;;) {
		node.spin(r2p::Time::ms(100));
		if (sub.fetch(msgp)) {
			palTogglePad(LED_GPIO, LED2);
			sub.release(*msgp);
		}
	}
	return CH_SUCCESS;
}

void rtcan_blinker(void) {
	switch (RTCAND1.state) {
	case RTCAN_MASTER:
		palClearPad(LED_GPIO, LED1);
		chThdSleepMilliseconds(200);
		palSetPad(LED_GPIO, LED1);
		chThdSleepMilliseconds(100);
		palClearPad(LED_GPIO, LED1);
		chThdSleepMilliseconds(200);
		palSetPad(LED_GPIO, LED1);
		chThdSleepMilliseconds(500);
		break;
	case RTCAN_SYNCING:
		palTogglePad(LED_GPIO, LED1);
		chThdSleepMilliseconds(100);
		break;
	case RTCAN_SLAVE:
		palTogglePad(LED_GPIO, LED1);
		chThdSleepMilliseconds(500);
		break;
	case RTCAN_ERROR:
		palTogglePad(LED_GPIO, LED4);
		chThdSleepMilliseconds(200);
		break;
	default:
		chThdSleepMilliseconds(100);
		break;
	}
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	r2p::Thread::set_priority(r2p::Thread::HIGHEST);
	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);

	rtcantra.initialize(rtcan_config);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 1, balance_node, NULL);

	r2p::Thread::set_priority(r2p::Thread::NORMAL);

	for (;;) {
		rtcan_blinker();
	}
	return CH_SUCCESS;
}
}
