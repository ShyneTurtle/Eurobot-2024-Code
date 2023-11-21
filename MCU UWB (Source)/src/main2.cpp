// #include <DW1000.h>

// DW1000 dw1000;

// void setup() {
//   Serial.begin(115200);
//   dw1000.begin();
// }

// void loop() {
//   // Configurer le mode du DW1000 pour l'émetteur (Tag)
//   dw1000.setDeviceMode(DW1000.MODE_SHORTDATA_FAST_ACCURACY);

//   // Mesurer la distance vers l'ancrage
//   uint16_t anchorAddress = 15;
//   dw1000.newTransmit();
//   dw1000.setDistantAddress(anchorAddress);
//   dw1000.startTransmit();
//   delay(10);  // Attendez un court instant pour la réponse de l'ancrage
// }
