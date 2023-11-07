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
#include "core0.hpp"

extern i2cSlaveData i2c_SD;
extern SignalProcessing signalM1;

int main()  {
    
    setup0();
    //launchCore1();

    // Indique la fin du setup
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);
    
    while(true) {
        //loop0();
        printf("nb de pas : %ld, pin0:%d\n", (int32_t)signalM1.absStep, gpio_get(PIN_M1_SA));
    }

    return 0;
}