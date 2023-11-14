// /!\ Pour upload sur Linux, taper "sudo chmod a+rw /dev/ttyACM0" dans le terminal

#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ranging.h>

// Pour le recepteur uniquement
// #include <Wire.h>
#include <Excel.hpp>
#include <mat.h>

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
// float trilateration(r1, r2, r3);

void setup()
{

    Serial.begin(115200);

    pinMode(33, INPUT);  // Diviseur de tension batterie
    pinMode(13, OUTPUT); // LED Batterie

    SPI.begin(18, 19, 23, 5); // Initialise SCLK sur pin 18, MISO sur pin 19, MOSI sur pin 23 et SS sur pin 5
    // Wire.setPins(21, 22);     // Initialise SDA sur pin 21 et SCL sur pin 22

    DW1000Ranging.initCommunication(14, 5, 15);         // Initialise les pins RST (14), IT (5) et SS (15) du module UWB
    DW1000Ranging.attachNewRange(newRange);             // Detecte si la distance entre une balise et un tag a change
    DW1000Ranging.attachNewDevice(newDevice);           // Detecte si un nouvelle appareil a été ajouter au réseau
    DW1000Ranging.attachInactiveDevice(inactiveDevice); // Detecte si un appareil est devenu inactif
    /*
    Initialisation de l'emetteur et le récepteur avec une adresse,
    ...en mode : `MODE_LONGDATA_RANGE_ACCURACY` (basically this is 110 kb/s data rate, 64 MHz PRF and long preambles),
    ...avec short adress non variable
    */
    DW1000Ranging.startAsTag(DEVICE_ADDRESS, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);

    uwb_data = init_link(); // Initialise le tableau
}
void loop()
{
    DW1000Ranging.loop(); // Lance l'acquisition des données UWB en boucle
    /*float r1 = MyLink.uwb_data[0]
    trilateration();*/
    newRange();

    dischargedBattery();
    veryDischargedBattery();
}

//=====Fonctions=====

void dischargedBattery()
{
    if ((float)analogRead(Diviseur_tension) / 1024.00 * 3.30 <= 3.05) // Allume Led si tension batterie inférieur à 12.3V
        digitalWrite(LED, HIGH);
}
void veryDischargedBattery()
{
    if ((float)analogRead(Diviseur_tension) / 1024.00 * 3.30 < 2.98)
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
    Serial.println(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX); // Ecrit l'adresse de l'emetteur qui a vu sa valeur change
    Serial.print("Distance : ");
    Serial.println(DW1000Ranging.getDistantDevice()->getRange()); // Ecrit sur le terminal la distance entre le recepepteur et l'emetteur ecrit ci-dessus

    update_link(uwb_data, DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange(), DW1000Ranging.getDistantDevice()->getRXPower());
    // Mets a jour le tableau uwb_data avec les nouvelles infos (adresses de l'appareil, distances ente appareil et recepteur, continuité du lien)
}
void newDevice(DW1000Device *device)
{
    Serial.print("Nouvel appareil : ");
    Serial.println(device->getShortAddress(), HEX); // Ecrit sur le terminal l'adresse du nouvel appareil

    add_link(uwb_data, device->getShortAddress()); // Ajoute le nouvelle appareil au tableau uwb_data
}
void inactiveDevice(DW1000Device *device)
{
    Serial.print("Nouvel appareil inactif : ");
    Serial.println(device->getShortAddress(), HEX); // Ecrit sur le terminal l'adresse de l'appareil devenu inactif

    delete_link(uwb_data, device->getShortAddress()); // Supprime l'appareil inactif du tableau uwb_data
}
/*float trilateration(r1, r2, r3){
    x = ((r1 * r1) - (r2 * r2) + (x2 * x2)) / (2 * x2);                            // Calcule la position en x du robot
    y = ((r1 * r1) - (r3 * r3) + (x3 * x3) + (y3 * y3) - (2 * x3 * x)) / (2 * x2); // Calcule la position en y du robot
}*/