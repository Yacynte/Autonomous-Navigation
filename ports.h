// Header guard
#ifndef PORTS_H
#define PORTS_H

#include "pico/stdlib.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Motor pins
#define Left_Motor_IN1 2
#define Left_Motor_IN2 3
#define Right_Motor_IN3 4
#define Right_Motor_IN4 5
#define ENA 6  // PWM pin for motor A
#define ENB 7  // PWM pin for motor B

//Ultrasonic Sensor pins
#define TRIGGER_PIN 12
#define ECHO_PIN 13

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

#define WIFI_SSID "Vodafone-AE61"
#define WIFI_PASSWORD "Opportunity2021"
#define SERVER_PORT 80
#define MAX_COMMAND_LEN 50

#endif 