MOTORBOARD MODULE
-----------------

This is a motor controller board that features an high power 24V, 20A H-bridge, current sense and quadrature encoder interface.
The MotorBoard module is supposed to work together with other smart modules, publishing data on CAN bus using a publish/subscriber QoS enabled middleware (under development).
The overall system should provide the devices and functionalities commonly used in robot prototyping, speeding up the development process and reducing the engineering time, leaving more time to research activities. The system is also supposed to enable students and amateurs to easily build robots without advanced hardware knowledge.

### Hardware

![Alt text](https://github.com/openrobots-dev/MotorBoard/hw/R2P_DCM_module.png)

The MotorBoard module is run by a STM32 ARM Cortex-M3 microcontroller that generates driving signals for the H-Bridge, monitors the current drawn by the motor and reads the incremental encoder feedback.
The microcontroller also runs a PID controller that can follow a position, speed or current (torque) set-point received on a specific topic over the CAN bus network.

The power stage of the MotorBoard features:

- four ST STD120N4F6 N-channel 40 V, 3.5 mOhm, 80A power MOSFETs
- two ST L6387E MOSFET drivers
- an Allegro ACS711 Hall-effect current sensor

Modules are connected together by a single cable, that transports CAN signals and 5V power. Each module has two ports, enabling chain connections.

A double connector footprint (RJ45 + pin header) is provided, so the user can choose which cabling fits better his needs:

- RJ45 has been chosen as it provides shielded twisted wires for 1Mbit CAN networking in harsh environments and cables are easy to find.
- pin header has been chosen to also give a space saving and simple connection when modules are close and RJ45 jack/plug pairing space is a problem. This header also enables using the IMU module outside our framework, accessing it through a simple TTL UART interface.

The four status LEDs are placed under RJ45 plug light pipes entrance, so it is only needed to solder the preferred connector without any other layout changes.

The left connector has the option to expose the TTL UART interface, instead of the CAN bus interface, by a solder jumper placed on the bottom of the board.

### Firmware

A demo firmware which tests the various hardware components is available.
The full firmware featuring closed loop control is under development.
