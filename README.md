# IKEA Fyrtyr roller blind WiFi module firmware

**Table of Contents**

[TOC]

## Introduction

This is a firmware for custom ESP32 / ESP8266 based WiFi module that can be used to control IKEA Fyrtyr and Kadrilj roller blinds.

First, why would one want to re-invent the wheel and replace the Ikea wireless module?

* Avoid finicky Zigbee pairing operation (first pair the remote with the hub, then with the signal repeater, then with the blinds and hope that everything turns out fine.. which hasn't been my experience with the crappy Ikea Smart app)
* Avoid unstable Zigbee normal mode operation (sometimes the blinds would go to deep sleep and would not wake up until manually woken up with a button press on the blinds. Also, the latency can vary from tolerable to awful)
* MQTT and Home Assistant support including MQTT auto-discovery functionality for automatic detection of Fyrtur nodes
* Possibility to use longer than 195cm blinds!
* Slower and quieter operation possible
* Possibility to integrate temperature/humidity sensors or maybe window break-in sensor?

There also exists [custom Fyrtur motor module firmware](https://github.com/mjuhanne/fyrtur-motor-board) that makes it possible to have more finer control of the motor unit:

 * **Allow setting custom motor speed.** The full speed with original FW is a bit too noisy for my ears, especially when used to control morning sunlight in the bedroom. Now it's possible to set the speed to 3 RPM and enjoy the completely silent operation, waking up to the sunlight instead of whirring noise :)
 * **Allow the use of 5-6 volt DC source.** Original firmware was intended to be used with rechargeable battery which was protected from under-voltage by ceasing operation when voltage drops below 6 VDC. Our custom firmware instead is recommended to be used with [custom Fyrtyr Wifi Module](https://github.com/mjuhanne/fyrtur-esp) (plugged to DC adapter) so the low voltage limit for motor operation can be ignored, mitigating the need to shop for the harder-to-get 6-7.5 volt adapters. The minimum operating voltage check can be enabled though if one wants to use this with the original Zigbee module with battery.
 * **Allow finer curtain position handling.** Original firmware has 1% granularity for the curtain position (0% - 100%). This translates to 1.5 cm resolution when using blinds with 1.5m tall window. It doesn't sound much, but when using sunlight-blocking curtain a lot of sunlight can seep between lower curtain and window board from a 0.5-1.5 cm gap. The custom firmware allows setting target position with sub-percent resolution so curtain can be lowered more accurately.

For more information about the custom firmware and the motor board itself (including reverse engineered schematics), please see the [motor module page](https://github.com/mjuhanne/fyrtur-motor-board).

## Wiring and installation

There are many ways one can install the module. Probably the easiest way is to modify the original main board:
 * Unsolder the battery power wiring and replace it with a connector with your own liking. A DC adapter has to be used since MQTT connection and sleep mode don't go so well together and the original battery will be soon empty. If using [custom Fyrtur motor module firmware](https://github.com/mjuhanne/fyrtur-motor-board) a 5 volt DC adapter can be used, otherwise the minimum operating voltage is around 6-6.5V.
 * Unsolder the Zigbee module. This can be achieved with a broad solder tip or with a desoldering station / hot air rework station. Care has to be taken not to inadvertently lift off the pads!

 ![Fyrtur main module wiring](images/fyrtur-main-board.png)

#### Custom PCB 

There is also a [custom PCB](https://github.com/mjuhanne/ikea-fyrtur) that replaces the original Fyrtur main board. It's been designed to hold ESP12F (ESP8266 PCB module) along with the usual reset paraphernalia. Alternatively one can use it as an interface board. In the latter case a separate ready-made ESP32 or ESP8266 module can be connected to the motor, LED and buttons via the interface board.

 ![Fyrtur custom PCB](https://github.com/mjuhanne/ikea-fyrtur/raw/main/images/Fyrtur-custom-PCB-1.png)

