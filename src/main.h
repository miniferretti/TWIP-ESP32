#ifndef _MAIN_H
#define _MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#include <VescUart.h>
#include <tasks.h>
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define SCL 22
#define SDA 21
#define OLED_ADD 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define BUZZ_PIN 2
#define BUTTON_PIN 4
#define BREAK_PIN 35
#define ACCEL_PIN 34
#define RXD2 16
#define TXD2 17
#define TASK_NUMBER 5
#define WIFI



#endif