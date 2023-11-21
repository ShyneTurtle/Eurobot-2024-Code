// #include <DW1000Ranging.h>

// DW1000Ranging = ranging;
// void measureDistanceWithAnchor(uint16_t anchorAddress);

// void setup() {
//   Serial.begin(115200);
//   ranging.begin();
// }

// void loop() {
//   // L'appareil interagit avec plusieurs balises
//   measureDistanceWithAnchor(/* Adresse de la première balise */);
//   delay(1000);  // Attendez une seconde entre chaque mesure
// }

// void measureDistanceWithAnchor(uint16_t anchorAddress) {
//   // Configurez le mode du DW1000 pour l'interaction avec la balise spécifiée
//   ranging.startRanging(anchorAddress);
//   delay(10);  // Attendez un court instant pour la réponse de la balise
//   if (ranging.isRangeComplete()) {
//     float distance = ranging.getRange();
//     Serial.print("Distance avec la balise ");
//     Serial.print(anchorAddress, HEX);
//     Serial.print(": ");
//     Serial.print(distance);
//     Serial.println(" m");
//   }
// }
