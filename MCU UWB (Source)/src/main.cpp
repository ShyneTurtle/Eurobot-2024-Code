#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ranging.h>

// Pin pour vérifier la charge de la batterie
#define Diviseur_tension 33
#define LED 13

// Adresse pour module UWB
#define DEVICE_ADDRESS "02:00:00:00:00:00:00:02" // Adresse des modules UWB dans les réseaux (UWB + ESP32)

struct MyLink *uwb_data; // Nomme le tableau uwb_data

//=====Fonctions=====
void dischargedBattery();
void veryDischargedBattery();
void newRange();
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);

void setup()
{

    Serial.begin(115200);

    pinMode(33, INPUT);  // Diviseur de tension batterie
    pinMode(13, OUTPUT); // LED Batterie

    SPI.begin(18, 19, 23, 5); // Initialise SCLK sur pin 18, MISO sur pin 19, MOSI sur pin 23 et SS sur pin 5

    DW1000Ranging.initCommunication(14, 5, 15);         // Initialise les pins RST (14), IT (5) et SS (15) du module UWB
    DW1000Ranging.attachNewRange(newRange);             // Detecte si la distance entre une balise et un tag a change
    DW1000Ranging.attachNewDevice(newDevice);           // Detecte si un nouvelle appareil a été ajouter au réseau
    DW1000Ranging.attachInactiveDevice(&inactiveDevice); // Detecte si un appareil est devenu inactif
    /*
    Initialisation de l'emetteur et le récepteur avec une adresse,
    ...en mode : `MODE_LONGDATA_RANGE_ACCURACY` (basically this is 110 kb/s data rate, 64 MHz PRF and long preambles),
    ...avec short adress fixe
    */
    DW1000Ranging.startAsAnchor(DEVICE_ADDRESS, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
}

void loop()
{
    DW1000Ranging.loop(); // Lance l'acquisition des données UWB en boucle

    dischargedBattery();
    veryDischargedBattery();
}

//=====Fonctions=====

void dischargedBattery()
{
    if (analogRead(Diviseur_tension) <= 4.92) // Allume Led si tension batterie inférieur à 12.3V
        digitalWrite(LED, HIGH);
}
void veryDischargedBattery()
{
    if (analogRead(Diviseur_tension) < 4.8)
    { // Fait clignoter la Led si tension batterie inférieur à 12V
        while (analogRead(Diviseur_tension) < 4.8)
        {
            digitalWrite(LED, HIGH);
            digitalWrite(LED, LOW);
        }
    }
}
void newRange()
{
    Serial.print("De : ");
    Serial.println(DW1000Ranging.getDistantDevice()->getShortAddress()); // Ecrit l'adresse de l'emetteur qui a vu sa valeur change
    Serial.print("Distance : ");
    Serial.println(DW1000Ranging.getDistantDevice()->getRange()); // Ecrit sur le terminal la distance entre le recepepteur et l'emetteur ecrit ci-dessus
}
void newDevice(DW1000Device *device)
{
    Serial.print("Nouvel appareil : ");
    Serial.println(device->getShortAddress(), HEX); // Ecrit sur le terminal l'adresse du nouvel appareil
}
void inactiveDevice(DW1000Device *device)
{
    Serial.print("Nouvel appareil inactif : ");
    Serial.println(device->getShortAddress(), HEX); // Ecrit sur le terminal l'adresse de l'appareil devenu inactif
}