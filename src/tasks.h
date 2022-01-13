#ifndef _TASKS_H
#define _TASKS_H

#include <Arduino.h>
#include <main.h>
#include <movingAvg.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <EEPROM.h>
#include <SimpleKalmanFilter.h>
#include <math.h>

#define BUTTON_QUEUE_LEN 1
#define VESCTOSCREEN_QUEUE_LEN 1
#define SCREENTOVESC_QUEUE_LEN 1
#define BREAK_QUEUE_LEN 1
#define ACCEL_QUEUE_LEN 1
#define VESC_IDL 127
#define VESC_MAX 255
#define VESC_MIN 0
#define WHEEL_RAD 0.125
#define POLE_PAIRS 7
#define REDUCTION_RATIO 0.089
#define MAX_CURRENT 150
#define MIN_CURRENT 0
#define MAX_BAT_VOLTAGE 50.4
#define MIN_BAT_VOLATGE 33.6
#define FIRMWARE_CREATOR "Matteo F."
#define FIRMWARE_VERSION 1

enum stateMachine
{
    BOOT,
    CRUSE,
    CALIBRATION,
    INFO,
    FLAPPY,
    ACCEL_BREAK_TEST
};

enum stateCalibration
{
    BEGIN,
    ACCEL_MAX,
    ACCEL_MIN,
    BREAK_MAX,
    BREAK_MIN,
    STORAGE,
    END
};

struct dataPackage
{
    float tempFET;
    float tempMotor;
    float avgMotorCurrent;
    float avgInputCurrent;
    float dutyCycleNow;
    long rpm;
    float inpVoltage;
    float ampHours;
    float ampHoursCharged;
    long tachometer;
    long tachometerAbs;
};

struct nunchuckPackage
{
    int valueX;
    int valueY;
    bool upperButton;
    bool lowerButton;
};

void screenOled_Task(void *paramter);

void vescControl_Task(void *parameter);

void button_Task(void *parameter);

void break_Task(void *parameter);

void accel_Task(void *parameter);

int calibration_routine(Adafruit_SH1106 *display, uint32_t *max_accel, uint32_t *min_accel, uint32_t *max_break, uint32_t *min_break);

#endif