// Le main contient uniquement les fonctions vers les fichiers
// Le programme est découpé en fichiers :
// main
//  --> picoIncludes
//  --> core0
//  --> core1
//  --> signalProcessing
//  --> lightMaths
//  --> speedCalculations
//  --> communications

#include "picoIncludes.hpp"
#include "communications.hpp"
#include "signalProcessing.hpp"
#include "motionControl.hpp"
#include "core0.hpp"

extern i2cSlaveData i2c_SD;
extern SignalProcessing signalM1;

int main()  {

    stdio_init_all();
    set_sys_clock_khz(CLOCK_FREQ_KHZ, 0);   // Définition de le vitesse d'horloge du proco
    
    setup0();
    launchCore1();

    // Indique la fin du setup
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);

    // Simulation d'un ordre provenant de l'I2C :
    int32_t valeur = 3500;
    i2c_SD.registers[0] = SET_SPEED_PID;
    i2c_SD.registers[1] = valeur>>24;
    i2c_SD.registers[2] = valeur>>16;
    i2c_SD.registers[3] = valeur>>8;
    i2c_SD.registers[4] = valeur;
    i2c_SD.new_data = true;
    
    while(true) {
        loop0();
        //printf("abs:%ld\n", (int32_t)signalM1.absStep);
    }

    return 0;
}