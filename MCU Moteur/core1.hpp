#pragma once

#include "picoIncludes.hpp"
#include "signalProcessing.hpp"
#include "communications.hpp"
#include "coreRoutine.hpp"

// Ces fonctions sont déclaré autre part
void interruptSignal2(uint gpio, uint32_t events);

Motor motor2;
MotionControl M2motion(MAX_SPEED * 0.7, 1);
extern SignalProcessing signalM2;

void setup1();
void loop1();

void core1()    {

    setup1();
    while(true) {

        loop1();
    }
    
}

void setup1()   {

    // === INITIALISATION MATERIELLE ===
    set_sys_clock_khz(CLOCK_FREQ_KHZ, 0);   // Définition de le vitesse d'horloge du proco

    // Active le timer système à la fréquence du core 
    systick_hw->csr |= 0x00000005;
    // On configure la valeur max pour avoir un temps de LOOP_TIME ms
    systick_hw->rvr  = (uint32_t)(LOOP_TIME * CLOCK_FREQ_KHZ);

    // Signal B
    gpio_init(PIN_M2_SB);
    gpio_set_dir(PIN_M2_SB, GPIO_IN);

    // Signal Dir
    gpio_init(PIN_M2_DIR);
    gpio_set_dir(PIN_M2_DIR, GPIO_OUT);

    // Fréquence d'exécution du code
    gpio_init(PIN_PROCO_1_FREQ);
    gpio_set_dir(PIN_PROCO_1_FREQ, GPIO_OUT);

    // PWM
    gpio_set_function(PIN_M2_PWM, GPIO_FUNC_PWM);
    uint pwm = pwm_gpio_to_slice_num(PIN_M2_PWM);
    pwm_set_wrap(pwm, PWM_WRAP_VALUE); // Set max
    pwm_set_chan_level(pwm, PWM_CHAN_A, 0); // 0%
    pwm_set_clkdiv(pwm, 25);
    pwm_set_enabled(pwm, 1);

    // Initialise les interruptions sur la broche du signal A
    gpio_set_irq_enabled_with_callback(PIN_M2_SA, GPIO_IRQ_EDGE_RISE, true, interruptSignal2);
    irq_set_priority(IO_IRQ_BANK0, 0);

    // Fin de setup
    multicore_fifo_push_blocking(1234);

}

void loop1()    {
    static bool beep = 0;
    static int last_core_order = 0;
    static int32_t last_core_value = 0;

    // === Comptage des fronts via les interruptions ===

    // Attend la fin du temps de loop
    while (!(systick_hw->csr & 0x00010000));

    // === calculs des données ===

    // Enregistre et remet à zéro le compteur de pas
    signalM2.buff_index = !signalM2.buff_index;
    // 500 impulsion par tours & rapport du réducteur 1/10 = 5000 impulsions par tours
    signalM2.speed_rotation = (signalM2.buff_count[!signalM2.buff_index] / PULSE_PER_TOUR) * (1000. / LOOP_TIME);
    signalM2.buff_count[!signalM2.buff_index] = 0;


    // Exécution des ordres du core 0 :
    if (listen(last_core_order, last_core_value)) {
        switch(last_core_order) {
        case MOVE:
            M2motion.move(last_core_value, signalM2.absStep);
            break;
        case ROTATE:
            M2motion.rotate(last_core_value, signalM2.absStep);
            break;
        case FORCE_STOP:
            M2motion.forceStop();
            break;
        }
    }

    // Calcul de la vitesse
    motor2.consigne = M2motion.getSpeedFromAdvancement(signalM2.absStep);
    // Asservissement vitesse
    routine(signalM2, motor2, 1);

    // === écriture de l'état des sorties ===  

    gpio_put(PIN_M2_DIR, motor2.direction);
    pwm_set_gpio_level(PIN_M2_PWM, motor2.pwm);

    // Permet de voir la fréquence d'exécution :
    beep = !beep;
    gpio_put(PIN_PROCO_1_FREQ, beep);
}