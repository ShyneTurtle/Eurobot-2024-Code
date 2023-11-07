#pragma once

#include "picoIncludes.hpp"
#include "lightMaths.hpp"

// Mesures de temps sur un encodeur et filtre median mobile
struct SignalProcessing  {
    volatile int32_t buff_count[2] = { 0, 0 };   // compteur de pas réinitialisé par le main
    volatile bool buff_index = 0;
    float speed_rotation = 0; // tr/s, la vitesse max du moteur est d'environ 25 tr/s
    volatile int64_t  absStep    = 0;      // Nombre de pas depuis le lancement
    volatile bool     signalB    = false;  // Niveau logique du signal B lors du déclenchement de l'interruption (indique le sens de rotation)
};

SignalProcessing signalM1;
SignalProcessing signalM2;

void interruptSignal1(uint gpio, uint32_t events) {
  // On regarde le sens de la marche moteur en regardant le signal B
  signalM1.signalB = gpio_get(PIN_M1_SB);

  // Ajoute un pas
  if (signalM1.signalB) {
    signalM1.buff_count[signalM1.buff_index] += 1;
    signalM1.absStep += 1;
  }
  else  {
    signalM1.buff_count[signalM1.buff_index] -= 1;
    signalM1.absStep -= 1;
  }
}

void interruptSignal2(uint gpio, uint32_t events) {
  // On regarde le sens de la marche moteur en regardant le signal B
  signalM2.signalB = gpio_get(PIN_M2_SB);

  // Ajoute un pas
  if (signalM2.signalB) {
    signalM2.buff_count[signalM2.buff_index] += 1;
    signalM2.absStep += 1;
  }
  else  {
    signalM2.buff_count[signalM2.buff_index] -= 1;
    signalM2.absStep -= 1;
  }
}