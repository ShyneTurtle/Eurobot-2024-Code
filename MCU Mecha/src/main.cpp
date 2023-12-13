#include "Arduino.h"
#include "Wire.h"
#include "ESP32Servo.h"

/**
 * ==== I2C MAP ====
 *
 * - RW/0x00 Grabber status (0 = open)
 * b0: Grabber1, b1: Grabber2, b2: Grabber3, b3: Grabber4, b4: Grabber5, b5: Grabber6
 *
 * - R/0x01 Infrared Barrier status (0 = nothing)
 * b0: Barrier1, b1: Barrier2, b2: Barrier3, b3: Barrier4, b4: Barrier5, b5: Barrier6
 *
 * - RW/0x02 Platform status
 * b0: Bottom Switch1, b1: Top Switch1, b2: Platform target1 (1=top), b3: Bottom Switch2, b4: Top Switch 2, b5: Platform target2(1=top)
 *
 * - RW/0x03
*/
byte i2c_reg[256] = { 0b111111, 0, 0b000000 };
byte i2c_target = 0;

void i2cReceive(int count) {
    bool reg_addr_byte = true;
    while (Wire.available()) {
        if (reg_addr_byte) {
            i2c_target = Wire.read();
            Serial.printf("I2C Addr set to: %d\n", i2c_target);
        }
        else {
            i2c_reg[i2c_target++] = Wire.read();
            Serial.printf("I2C W @%d, Data: %d\n", i2c_target-1, i2c_reg[i2c_target-1]);
        }
        reg_addr_byte = false;
    }
}

void i2cRequest() {
    Wire.write(i2c_reg[i2c_target++]);
    Serial.printf("I2C R @%d, Data: %d\n", i2c_target-1, i2c_reg[i2c_target-1]);
}

// === Barrier ===
byte barrier_pins[6] = {
    35,
    13,
    14,
    15,
    16,
    39
};
byte* barrier_reg = i2c_reg + 1;

// === Grabber ===
const int GRABBER_ANGLE_OPENED[] = {164,170,152,164,170,152};
const int GRABBER_ANGLE_PLANT[] = {135,139,122,135,139,122};
const int GRABBER_ANGLE_CUP[] = {150,155,140,150,155,140};

byte servo_pins[6] = {
    33,
    26,
    27,
    5,
    19,
    23
};
Servo servo_list[6];
byte* grabber_reg = i2c_reg;

// === Platform ===
#define PLATFORM_SPEED 255
#define PLATFORM1_BOTTOM_PIN 32
#define PLATFORM1_TOP_PIN 25
#define PLATFORM1_MF_PIN 2
#define PLATFORM1_MR_PIN 4
#define PLATFORM1_BOTTOM_BIT 0
#define PLATFORM1_TOP_BIT 1
#define PLATFORM1_TARGET_BIT 2
#define PLATFORM2_BOTTOM_PIN 17
#define PLATFORM2_TOP_PIN 18
#define PLATFORM2_MF_PIN 16
#define PLATFORM2_MR_PIN 12
#define PLATFORM2_BOTTOM_BIT 3
#define PLATFORM2_TOP_BIT 4
#define PLATFORM2_TARGET_BIT 5
byte* platform_reg = i2c_reg + 2;


void setup() {
    Wire.begin(0x27);
    Wire.onReceive(i2cReceive);
    Wire.onRequest(i2cRequest);

    Serial.begin(115200);

    for (byte bit = 0; bit < 6; bit++) {
        // Barriers
        pinMode(barrier_pins[bit], INPUT);
        // Grabbers
        servo_list[bit].attach(servo_pins[bit]);
    }

    pinMode(PLATFORM1_BOTTOM_PIN, INPUT);
    pinMode(PLATFORM1_TOP_PIN, INPUT);
    pinMode(PLATFORM1_MF_PIN, OUTPUT);
    pinMode(PLATFORM1_MR_PIN, OUTPUT);
    pinMode(PLATFORM2_BOTTOM_PIN, INPUT);
    pinMode(PLATFORM2_TOP_PIN, INPUT);
    pinMode(PLATFORM2_MF_PIN, OUTPUT);
    pinMode(PLATFORM2_MR_PIN, OUTPUT);
}

void loop() {
    for (byte bit = 0; bit < 6; bit++) {
        // === Barriers ===
        // Set to 0 the current bit
        (*barrier_reg) &= ~(1 << bit);
        // Set to 1 the current bit if something is detected
        (*barrier_reg) |= (!digitalRead(barrier_pins[bit])) << bit;

        // === Grabbers ===
        // Get the target angle based on wether the grabber should be opened or closed
        int angle = (*grabber_reg) & (1 << bit) ? GRABBER_ANGLE_PLANT[bit] : GRABBER_ANGLE_OPENED[bit];
        servo_list[bit].write(angle);
    }

    // === Platform1 elevation ===
    // Read limit switches
    bool platform1_bottom = !digitalRead(PLATFORM1_BOTTOM_PIN);
    bool platform1_top = !digitalRead(PLATFORM1_TOP_PIN);
    // Read target from I2C reg
    bool platform1_target = (*platform_reg) & (1 << PLATFORM1_TARGET_BIT);
    // Make the motor move to the target position
    // Set the H Bridge mode on the correct pin depending on the rotation direction
    int platform1_mode_pin = platform1_target ? PLATFORM1_MR_PIN : PLATFORM1_MF_PIN;
    digitalWrite(platform1_mode_pin, 0);
    // Send a PWM to the H Bridge
    int platform1_pwm_pin = platform1_target ? PLATFORM1_MF_PIN : PLATFORM1_MR_PIN;
    analogWrite(
        platform1_pwm_pin,
        (platform1_target ? !platform1_top : !platform1_bottom) * PLATFORM_SPEED
    );
    // Save reads in the I2C register
    (*platform_reg) &= ~(1 << PLATFORM1_BOTTOM_BIT);
    (*platform_reg) |= platform1_bottom << PLATFORM1_BOTTOM_BIT;
    (*platform_reg) &= ~(1 << PLATFORM1_TOP_BIT);
    (*platform_reg) |= platform1_top << PLATFORM1_TOP_BIT;


    // === Platform2 elevation ===
    // Read limit switches
    bool platform2_bottom = !digitalRead(PLATFORM2_BOTTOM_PIN);
    bool platform2_top = !digitalRead(PLATFORM2_TOP_PIN);
    // Read target from I2C reg
    bool platform2_target = (*platform_reg) & (1 << PLATFORM2_TARGET_BIT);
    // Make the motor move to the target position
    // Set the H Bridge mode on the correct pin depending on the rotation direction
    int platform2_mode_pin = platform2_target ? PLATFORM2_MR_PIN : PLATFORM2_MF_PIN;
    digitalWrite(platform2_mode_pin, 1);
    // Send a PWM to the H Bridge
    int platform2_pwm_pin = platform2_target ? PLATFORM2_MF_PIN : PLATFORM2_MR_PIN;
    analogWrite(
        platform2_pwm_pin,
        (platform2_target ? !platform2_top : !platform2_bottom) * PLATFORM_SPEED
    );
    // Save reads in the I2C register
    (*platform_reg) &= ~(1 << PLATFORM2_BOTTOM_BIT);
    (*platform_reg) |= platform2_bottom << PLATFORM2_BOTTOM_BIT;
    (*platform_reg) &= ~(1 << PLATFORM2_TOP_BIT);
    (*platform_reg) |= platform2_top << PLATFORM2_TOP_BIT;

    Serial.printf("Pinces: %x, Barrieres: %x, Platformes: %x\n", i2c_reg[0], i2c_reg[1], i2c_reg[2]);
}