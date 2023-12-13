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
byte i2c_reg[256] = { 0 };
byte i2c_target = 0;
void i2cReceive(int count) {
    bool reg_addr_byte = true;
    while (Wire.available()) {
        if (reg_addr_byte)
            i2c_target = Wire.read();
        else
            i2c_reg[i2c_target++] = Wire.read();
        reg_addr_byte = false;
    }
    i2c_reg[i2c_target++] = Wire.read();
}
void i2cRequest() {
    Wire.write(i2c_reg[i2c_target++]);
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
#define GRABBER_ANGLE_OPENED 120
#define GRABBER_ANGLE_CLOSED 200
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
#define PLATFORM_SPEED 100
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
        int angle = (*grabber_reg) & (1 << bit) ? GRABBER_ANGLE_CLOSED : GRABBER_ANGLE_OPENED;
        servo_list[bit].write(angle);
    }

    // === Platform1 elevation ===
    // Read limit switches
    bool platform1_bottom = digitalRead(PLATFORM1_BOTTOM_PIN);
    bool platform1_top = digitalRead(PLATFORM1_TOP_PIN);
    // Read target from I2C reg
    bool platform1_target = (*platform_reg) & (1 << PLATFORM1_TARGET_BIT);
    // Make the motor move to the target position
    digitalWrite(PLATFORM1_MR_PIN, !platform1_target);
    analogWrite(
        PLATFORM1_MF_PIN,
        (platform1_target ? platform1_top : platform1_bottom) * PLATFORM_SPEED
    );
    // Save reads in the I2C register
    (*platform_reg) &= ~(1 << PLATFORM1_BOTTOM_BIT);
    (*platform_reg) |= platform1_bottom << 0;
    (*platform_reg) &= ~(1 << PLATFORM1_TOP_BIT);
    (*platform_reg) |= platform1_top << PLATFORM1_TOP_BIT;


    // === Platform2 elevation ===
    // Read limit switches
    bool platform2_bottom = digitalRead(PLATFORM2_BOTTOM_PIN);
    bool platform2_top = digitalRead(PLATFORM2_TOP_PIN);
    // Read target from I2C reg
    bool platform2_target = (*platform_reg) & (1 << PLATFORM2_TARGET_BIT);
    // Make the motor move to the target position
    digitalWrite(PLATFORM2_MR_PIN, !platform2_target);
    analogWrite(
        PLATFORM2_MF_PIN,
        (platform2_target ? platform2_top : platform2_bottom) * PLATFORM_SPEED
    );
    // Save reads in the I2C register
    (*platform_reg) &= ~(1 << PLATFORM2_BOTTOM_BIT);
    (*platform_reg) |= platform2_bottom << 0;
    (*platform_reg) &= ~(1 << PLATFORM2_TOP_BIT);
    (*platform_reg) |= platform2_top << PLATFORM2_TOP_BIT;

    Serial.printf("Pinces: %x, Barrieres: %x, Platformes: %x\n", i2c_reg[0], i2c_reg[1], i2c_reg[2]);
}