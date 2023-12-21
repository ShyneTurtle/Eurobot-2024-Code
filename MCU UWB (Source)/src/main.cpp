// This program calibrates an ESP32_UWB module intended for use as a fixed anchor point
// uses binary search to find anchor antenna delay to calibrate against a known distance
//
// modified version of Thomas Trojer's DW1000 library is required!

// Remote tag (at origin) must be set up with default antenna delay (library default = 16384)

// user input required, possibly unique to each tag:
// 1) accurately measured distance from anchor to tag
// 2) address of anchor
//
// output: antenna delay parameter for use in final anchor setup.

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// ESP32_UWB pin definitions

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

// connection pins
const uint8_t PIN_RST = 14; // reset pin
const uint8_t PIN_IRQ = 15; // irq pin
const uint8_t PIN_SS = 5;   // spi select pin

// char this_anchor_addr[] = "02:00:00:00:00:00:00:01";
// char this_anchor_addr[] = "02:00:00:00:00:00:00:02";
char this_anchor_addr[] = "02:00:00:00:00:00:00:03";
float this_anchor_target_distance = 1; // measured distance to anchor in m

uint16_t this_anchor_Adelay = 16600; // starting value
uint16_t Adelay_delta = 100;         // initial binary search step size

void veryDischargedBattery();
void veryDischargedBattery();
void newRange();
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);

// For the battery
#define Diviseur_tension 33
#define LED 13
unsigned long int lastMillis;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    // init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pin

    Serial.print("Starting Adelay ");
    Serial.println(this_anchor_Adelay);
    Serial.print("Measured distance ");
    Serial.println(this_anchor_target_distance);

    DW1000.setAntennaDelay(this_anchor_Adelay);

    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    // Enable the filter to smooth the distance
    // DW1000Ranging.useRangeFilter(true);

    // start the module as anchor, don't assign random short address
    DW1000Ranging.startAsAnchor(this_anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop()
{
    DW1000Ranging.loop();

    // dischargedBattery();
    // veryDischargedBattery();
}

void dischargedBattery()
{
    if ((float)analogRead(Diviseur_tension) / 1024.00 * 3.30 <= 3.05) // Led ON if Vbat<12.3V
        digitalWrite(LED, HIGH);
}
void veryDischargedBattery()
{
    if ((float)analogRead(Diviseur_tension) / 1024.00 * 3.30 < 2.98)
    { // Led blink when Vbat < 12V
        if ((millis() >> 9) % 2)
        { // Change Led state every 512ms
            digitalWrite(LED, !digitalRead(LED));
        }
    }
}
void newRange()
{
    static float last_delta = 0.0;
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC);

    float dist = DW1000Ranging.getDistantDevice()->getRange();

    Serial.print(",");
    Serial.print(dist);
    if (Adelay_delta < 3)
    {
        Serial.print(", final Adelay ");
        Serial.println(this_anchor_Adelay);
        //    Serial.print("Check: stored Adelay = ");
        //    Serial.println(DW1000.getAntennaDelay());
        // while (1)
        // {
        //     Serial.print("from: ");
        //     Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
        //     // Serial.print("\n");
        //     Serial.print("\t Range: ");
        //     Serial.print(DW1000Ranging.getDistantDevice()->getRange());
        //     Serial.println(" m");
        // }; // done calibrating
    }

    float this_delta = dist - this_anchor_target_distance; // error in measured distance

    if (this_delta * last_delta < 0.0)
        Adelay_delta = Adelay_delta / 2; // sign changed, reduce step size
    last_delta = this_delta;

    if (this_delta > 0.0)
        this_anchor_Adelay += Adelay_delta; // new trial Adelay
    else
        this_anchor_Adelay -= Adelay_delta;

    Serial.print(", Adelay = ");
    Serial.println(this_anchor_Adelay);
    //  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
    DW1000.setAntennaDelay(this_anchor_Adelay);
}

void newDevice(DW1000Device *device)
{
    Serial.print("Device added: ");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}