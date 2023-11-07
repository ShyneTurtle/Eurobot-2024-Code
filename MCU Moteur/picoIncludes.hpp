#pragma once

#include <stdio.h>
#include <pico/time.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/i2c_slave.h>
#include <hardware/pwm.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include <hardware/exception.h>
#include <hardware/structs/systick.h>

//Définitions générales :
#define FORWARD     1     // Marche avant
#define BACKWARD    0     // Marche arrière

#define CLOCK_FREQ_KHZ  100000  // Fréquence du rp2040 (kHz)
#define PWM_WRAP_VALUE  255     // Précision du pwm (1 -> 0xffff), influe aussi sur la fréquence du pwm
#define MAX_SPEED       4.5
#define LOOP_TIME       10      // Temps d'une loop en ms
#define PULSE_PER_TOUR  15000.  // Nombre d'impulsion encodeur par tour de roue (10 * 500 * 3)
#define WHEEL_PERIMETER 20      // Périmètre de la roue (en cm)

// I2C :
#define PIN_I2C_SDA         16
#define PIN_I2C_SCL         17
#define I2C_SLAVE_ADDRESS   0x17
#define I2C_BAUDRATE        100000  // Vitesse de transmission I2C (Hz)

// Moteur 1
#define PIN_M1_SA         0     // Pin du signal A, moteur 1
#define PIN_M1_SB         1     // Pin du signal B, moteur 1
#define PIN_PROCO_0_FREQ  10    // Pin indiquant la fréquence/2 du proco 0
#define PIN_M1_PWM        5    // Pin de commande PWM, moteur 1
#define PIN_M1_DIR        6    // Pin de direction moteur, moteur 1

// Moteur 2
#define PIN_M2_SA         2     // Pin du signal A moteur 2
#define PIN_M2_SB         3     // Pin du signal B moteur 2
#define PIN_PROCO_1_FREQ  11    // Pin indiquant la fréquence/2 du proco 1
#define PIN_M2_PWM        14    // Pin de commande PWM, moteur 2
#define PIN_M2_DIR        15     // Pin de direction moteur, moteur 2