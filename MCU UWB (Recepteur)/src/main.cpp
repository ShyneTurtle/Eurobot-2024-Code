#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <string>
#include <map>
#include <vector>

#define MOY_COUNT 10
std::map<uint16_t, std::vector<double>> ranges;
void rangePush(DW1000Device *device) // place last distance measure on a 100 values string on increasing order
{
    ranges[device->getShortAddress()].push_back(device->getRange());
    if (ranges.size() > MOY_COUNT)
        ranges.erase(ranges.begin());
}
double rangeMed(DW1000Device *device) // Calcule of mediane of 100 last distances
{
    std::vector<double> sorted_list = ranges[device->getShortAddress()];
    std::sort(sorted_list.begin(), sorted_list.end());

    if (sorted_list.size() % 2)
        return (sorted_list[sorted_list.size() / 2] + sorted_list[(sorted_list.size() / 2) - 1]) / 2;

    return sorted_list[sorted_list.size() / 2]; // Return mediane of 100 last distances
}

// #define DEBUG_TRILAT // Prints in trilateration code
// #define DEBUG_DIST // Print anchor distances

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

// Connection pins
const uint8_t PIN_RST = 14; // Reset pin
const uint8_t PIN_IRQ = 15; // Irq pin
const uint8_t PIN_SS = 5;   // SPI select pin

// TAG antenna delay defaults to 16384
// Leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

// Variables for position determination
#define N_ANCHORS 3
#define ANCHOR_DISTANCE_EXPIRED 5000 // Measurements older than this are ignore (milliseconds)

// Global variables, input and output

float anchor_matrix[N_ANCHORS][3] = {
    // List of anchor coordinates, relative to chosen origin.
    // First column : x, Second : Y and Third : Z
    {0.0, 0.0, 0}, // Anchor labeled #1
    {2.0, 0.0, 0}, // Anchor labeled #2
    {1.0, 3.0, 0}, // Anchor labeled #3
};                 // Z values are ignored in this code

uint32_t last_anchor_update[N_ANCHORS] = {0};  // Millis() value last time anchor was seen
float last_anchor_distance[N_ANCHORS] = {0.0}; // Most recent distance reports

float current_tag_position[2] = {0.0, 0.0}; // Global current position (meters with respect to anchor origin)
float current_distance_rmse = 0.0;          // RMS error in distance calc => crude measure of position error (meters).  Needs to be better characterized

// void veryDischargedBattery();
// void veryDischargedBattery();
void newRange();
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
int trilat2D_3A();
void i2cReceive(int count);
void i2cRequest();

// Pin for the battery
// #define Diviseur_tension 33
// #define LED 13
// unsigned long int lastMillis;

byte i2c_reg[256] = {0};
byte i2c_target = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Set up I2C
    Wire.begin(0x11);
    Wire.onRequest(i2cRequest);
    Wire.onReceive(i2cReceive);

    // Initialize configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pin

    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    // Start as tag, do not assign random short address
    DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

    // dischargedBattery();
    // veryDischargedBattery();
}

void loop()
{
    DW1000Ranging.loop();

    float x = current_tag_position[0] * 100;
    float y = current_tag_position[1] * 100;

    i2c_reg[0] = (unsigned char)x;
    i2c_reg[1] = ((unsigned short)y) >> 8;
    i2c_reg[2] = ((unsigned short)y);
}

// void dischargedBattery()
// {
//     if ((float)analogRead(Diviseur_tension) / 1024.00 * 3.30 <= 3.05) // Led ON if Vbat<12.3V
//         digitalWrite(LED, HIGH);
// }
// void veryDischargedBattery()
// {
//     if ((float)analogRead(Diviseur_tension) / 1024.00 * 3.30 < 2.98)
//     { // Led blink if Vbat < 12V
//         if ((millis() >> 9) % 2)
//         { // Change Led state every 512ms
//             digitalWrite(LED, !digitalRead(LED));
//         }
//     }
// }
// Collect distance data from anchors, presently configured for 4 anchors
// Solve for position if all four current
void newRange()
{
    rangePush(DW1000Ranging.getDistantDevice());

    int i; // Indices, expecting values 1 to 4
    // Index of this anchor, expecting values 1,2,3
    int index = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x03;

    // Find the good antenna delay
    if (index > 0)
    {
        last_anchor_update[index - 1] = millis();                 // Decrement for array index
        float range = rangeMed(DW1000Ranging.getDistantDevice()); // DW1000Ranging.getDistantDevice()->getRange();
        last_anchor_distance[index - 1] = range;
        if (range < 0.0 || range > 30.0)
            last_anchor_update[index - 1] = 0; // Sanity check, ignore this measurement
    }

    int detected = 0;

    // Reject old measurements
    for (i = 0; i < N_ANCHORS; i++)
    {
        if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED)
            last_anchor_update[i] = 0; // Not from this one
        if (last_anchor_update[i] > 0)
            detected++;
    }

    if (detected == 3)
    { // Three measurements TODO: check millis() wrap

#ifdef DEBUG_DIST
      // Print distance and age of measurement
        uint32_t current_time = millis();
        for (i = 0; i < N_ANCHORS; i++)
        {
            Serial.print(last_anchor_distance[i]);
            Serial.print("\t");
            Serial.println(current_time - last_anchor_update[i]); // Age in millis
        }
#endif

        trilat2D_3A();

        // Output the values (X, Y and error estimate)
        Serial.print("P = ");
        Serial.print(current_tag_position[0]);
        Serial.write(',');
        Serial.print(current_tag_position[1]);
        Serial.write(',');
        Serial.println(current_distance_rmse);
    }
} // End newRange
void newDevice(DW1000Device *device) // Print address of new devices
{
    Serial.print("Device added: ");
    Serial.println(device->getShortAddress(), HEX);
}
void inactiveDevice(DW1000Device *device) // Print address of inactive device
{
    Serial.print("Delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}
int trilat2D_3A(void)
{
    /*
    For method see technical paper at
    https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf

    A nice feature of this method is that the normal matrix depends only on the anchor arrangement
    and needs to be inverted only once. Hence, the position calculation should be robust.
    */
    static bool first = true;         // First time through, some preliminary work
    float b[N_ANCHORS], d[N_ANCHORS]; // Temp vector, distances from anchors

    static float Ainv[2][2], k[N_ANCHORS]; // These are calculated only once

    int i;
    // Copy distances to local storage
    for (i = 0; i < N_ANCHORS; i++)
        d[i] = last_anchor_distance[i];

#ifdef DEBUG_TRILAT
    char line[60];
    snprintf(line, sizeof line, "d: %6.2f %6.2f %6.2f", d[0], d[1], d[2]);
    Serial.println(line);
#endif

    if (first)
    { // Intermediate fixed vectors
        first = false;

        float x[N_ANCHORS], y[N_ANCHORS]; // Intermediate vectors
        float A[2][2];                    // The A matrix for system of equations to solve

        for (i = 0; i < N_ANCHORS; i++)
        {
            x[i] = anchor_matrix[i][0];
            y[i] = anchor_matrix[i][1];
            k[i] = x[i] * x[i] + y[i] * y[i];
        }

        // Set up least squares equation

        for (i = 1; i < N_ANCHORS; i++)
        {
            A[i - 1][0] = x[i] - x[0];
            A[i - 1][1] = y[i] - y[0];
#ifdef DEBUG_TRILAT
            snprintf(line, sizeof line, "A  %5.2f %5.2f \n", A[i - 1][0], A[i - 1][1]);
            Serial.println(line);
#endif
        }
        // Invert A
        float det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
        if (fabs(det) < 1.0E-4)
        {
            Serial.println("***Singular matrix, check anchor coordinates***");
            while (1)
                delay(1); // Hang
        }

#ifdef DEBUG_TRILAT
        snprintf(line, sizeof line, "det A %8.3e\n", det);
        Serial.println(line);
#endif

        det = 1.0 / det;
        // Scale adjoint
        Ainv[0][0] = det * A[1][1];
        Ainv[0][1] = -det * A[0][1];
        Ainv[1][0] = -det * A[1][0];
        Ainv[1][1] = det * A[0][0];
    } // End if (first);

    for (i = 1; i < N_ANCHORS; i++)
    {
        b[i - 1] = d[0] * d[0] - d[i] * d[i] + k[i] - k[0];
    }

    // Least squares solution for position
    // Solve:  2 A rc = b

    current_tag_position[0] = 0.5 * (Ainv[0][0] * b[0] + Ainv[0][1] * b[1]);
    current_tag_position[1] = 0.5 * (Ainv[1][0] * b[0] + Ainv[1][1] * b[1]);

    // Calculate RMS error for distances
    float rmse = 0.0, dc0 = 0.0, dc1 = 0.0;
    for (i = 0; i < N_ANCHORS; i++)
    {
        dc0 = current_tag_position[0] - anchor_matrix[i][0];
        dc1 = current_tag_position[1] - anchor_matrix[i][1];
        dc0 = d[i] - sqrt(dc0 * dc0 + dc1 * dc1);
        rmse += dc0 * dc0;
    }
    current_distance_rmse = sqrt(rmse / ((float)N_ANCHORS));

    return 1;
} // End trilat2D_3A
void i2cReceive(int count)
{
    bool reg_addr_byte = true;
    while (Wire.available())
    {
        if (reg_addr_byte)
        {
            i2c_target = Wire.read();
        }
        else
        {
            i2c_reg[i2c_target++] = Wire.read();
        }
        reg_addr_byte = false;
    }
}
void i2cRequest()
{
    Wire.write(i2c_reg[i2c_target++]);
}