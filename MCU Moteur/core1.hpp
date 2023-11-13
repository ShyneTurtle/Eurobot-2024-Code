#pragma once

#include "picoIncludes.hpp"
#include "signalProcessing.hpp"
#include "communications.hpp"
#include "motorControl.hpp"

// Ces fonctions sont déclaré autre part
void interruptSignal2(uint gpio, uint32_t events);

Motor motor2;
MotionControl M2motion(MAX_SPEED, 1);
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

    // Active le timer système à la fréquence du core 
    systick_hw->csr |= 0x00000005;
    // On configure la valeur max pour avoir un temps de LOOP_TIME ms
    systick_hw->rvr  = (uint32_t)(LOOP_TIME * CLOCK_FREQ_KHZ);

    // Signal A
    // gpio_init(PIN_M2_SA);
    // gpio_set_dir(PIN_M2_SA, GPIO_IN);

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
    pwm_set_clkdiv(pwm, 15);
    pwm_set_enabled(pwm, 1);

    // Initialise les interruptions sur la broche du signal A
    gpio_set_irq_enabled_with_callback(PIN_M2_SA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, interruptSignal2);
    // gpio_set_irq_enabled_with_callback(PIN_M2_SB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, interruptSignal2);
    irq_set_priority(IO_IRQ_BANK0, 0);

    setPIDcoeff(motor2.pid_position_control, 0.001, 0.0005, 0);
    setPIDcoeff(motor2.pid_speed_control, 1, 0.1, 0);

    M2motion.setOrder(FORCE_STOP, 0, 0);
    // Fin de setup
    multicore_fifo_push_blocking(1234);
}

void loop1()    {
    static uint8_t last_core_order = 0;
    static int32_t last_core_value = 0;
 
    // === Comptage des fronts via les interruptions ===

    // On vérifie que le timer ne soit pas déjà terminé (cela ne doit pas arriver)
    if (systick_hw->csr & 0x00010000)   {
        gpio_put(PIN_PROCO_1_FREQ, 1);
        // Générer un message d'alerte ?
    }
    else    {
        // Attend la fin du temps de loop
        gpio_put(PIN_PROCO_1_FREQ, 0);  // Temps d'attente
        while (!(systick_hw->csr & 0x00010000));
        gpio_put(PIN_PROCO_1_FREQ, 1);  // Temps de travail
    }    

    // === calculs des données ===

    signalM2.buff_count[!signalM2.buff_index] = 0;  // Reset de l'ancien buffer
    signalM2.buff_index = !signalM2.buff_index;     // Changement de buffer
    // nb de tour * temps
    signalM2.speed_rotation = (signalM2.buff_count[!signalM2.buff_index] / (float)PULSE_PER_TOUR) * (1000. / LOOP_TIME);
    // Ajout du nombre de pas sur l'absolu
    signalM2.absStep += signalM2.buff_count[!signalM2.buff_index];
    // Calcul de la vitesse
    M2motion.computeSpeedFromAdvancement(signalM2, motor2);

    // === ordre ===

    // Exécution des ordres du core 0 :
    if (listen(last_core_order, last_core_value)) {
        printf("send : o:%d, v:%ld", last_core_order, last_core_value);
        M2motion.setOrder(last_core_order, last_core_value, signalM2.absStep);
    }

    // === écriture de l'état des sorties ===  

    gpio_put(PIN_M2_DIR, motor2.direction);
    pwm_set_gpio_level(PIN_M2_PWM, motor2.pwm);
}