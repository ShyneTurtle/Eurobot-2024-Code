#include <string>
#include <map>
#include <vector>
#include <SPI.h>
#include <Wire.h>
#include "DW1000Ranging.h"

#define MOY_COUNT 10
std::map<uint16_t, std::vector<double>> ranges;
void rangePush(DW1000Device *device)
{
  ranges[device->getShortAddress()].push_back(device->getRange());
  if (ranges.size() > MOY_COUNT)
    ranges.erase(ranges.begin());
}
double rangeMed(DW1000Device *device)
{
  std::vector<double> sorted_list = ranges[device->getShortAddress()];
  std::sort(sorted_list.begin(), sorted_list.end());

  if (sorted_list.size() % 2)
    return (sorted_list[sorted_list.size() / 2] + sorted_list[(sorted_list.size() / 2) - 1]) / 2;

  return sorted_list[sorted_list.size() / 2];
}

void dischargedBattery();
void veryDischargedBattery();
void newRange();
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);

#define Diviseur_tension 33
#define LED 13

void setup()
{
  Serial.begin(115200);

  SPI.begin(18, 19, 23, 5);

  // init the configuration
  DW1000Ranging.initCommunication(14, 5, 15); // Reset, CS, IRQ pin
  // define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  // Enable the filter to smooth the distance
  DW1000Ranging.useRangeFilter(false);

  // we start the module as a tag
  //  DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_SHORTDATA_FAST_ACCURACY, false);
  DW1000Ranging.startAsTag("02:00:00:00:00:00:00:01", DW1000.MODE_LONGDATA_RANGE_ACCURACY, 0);
}

void loop()
{
  DW1000Ranging.loop();

  /*
  Stockage des medianes dans un tableau :
  Ligne 1 : module UWB qui a pour adresse 0
  Ligne 2 : module UWB qui a pour adresse 1
  Ligne 3 : module UWB qui a pour adresse 2
   */

  float TabVal1[3];
  float x, y;
  char addr = DW1000Ranging.getDistantDevice()->getShortAddress();

  Serial.printf("%d \n", addr);

  TabVal1[addr] = rangeMed(DW1000Ranging.getDistantDevice());

  trilateration(TabVal1, &x, &y);
  Serial.printf("Coordonnée en x : %f et coordonnée en y : %f\n", x, y);

  dischargedBattery();
  veryDischargedBattery();
}

//====Fonctions====

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
  // Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  // Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
  // Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
  rangePush(DW1000Ranging.getDistantDevice());
  Serial.printf("Mediane: %f", rangeMed(DW1000Ranging.getDistantDevice()));
}
void newDevice(DW1000Device *device)
{
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}
void inactiveDevice(DW1000Device *device)
{
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
void trilateration(float r[], float *x, float *y)
{
  *x = (r[0] * r[0] - r[1] * r[1] + 40000) / 400;
  *y = (r[0] * r[0] - r[2] * r[2] + 100000 - 200 * *x) / 600;
}