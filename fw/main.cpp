#include "ch.h"
#include "hal.h"

#include "config.h"

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

#include "r2p/node/led.hpp"
#include "r2p/node/motor.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

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

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

// RTCAN transport
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };


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

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO, r2p::ledsub_node, NULL);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 1, r2p::qeipub_node, NULL);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 2, r2p::pwm2sub_node, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}
	return CH_SUCCESS;
}
}
