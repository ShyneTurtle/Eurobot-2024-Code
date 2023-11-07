#pragma once

#include "picoIncludes.hpp"
#include "signalProcessing.hpp"
#include "coreRoutine.hpp"
#include "core1.hpp"
#include "communications.hpp"


// Ces fonctions sont déclaré autre part
void interruptSignal1(uint gpio, uint32_t events);
void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event);
void core1();

Motor motor1;
extern SignalProcessing signalM1;
extern i2cSlaveData i2c_SD;
MotionControl M1motion(MAX_SPEED * 0.7, 0);
uint32_t fifo_value = 0;

void setup0()   {

    // === INITIALISATION MATERIELLE ===
    stdio_init_all();
    set_sys_clock_khz(CLOCK_FREQ_KHZ, 0);   // Définition de le vitesse d'horloge du proco

    // Active le timer système à la fréquence du core 
    systick_hw->csr |= 0x00000005;
    // On configure la valeur max pour avoir un temps de LOOP_TIME ms (ms * khz)
    systick_hw->rvr  = (uint32_t)(LOOP_TIME * CLOCK_FREQ_KHZ);

    // Signal B
    gpio_init(PIN_M1_SB);
    gpio_set_dir(PIN_M1_SB, GPIO_IN);

    // Signal Dir
    gpio_init(PIN_M1_DIR);
    gpio_set_dir(PIN_M1_DIR, GPIO_OUT);

    // Fréquence d'exécution du code
    gpio_init(PIN_PROCO_0_FREQ);
    gpio_set_dir(PIN_PROCO_0_FREQ, GPIO_OUT);

    // PWM
    gpio_set_function(PIN_M1_PWM, GPIO_FUNC_PWM);
    uint pwm = pwm_gpio_to_slice_num(PIN_M1_PWM);
    pwm_set_wrap(pwm, PWM_WRAP_VALUE); // Set max
    pwm_set_chan_level(pwm, PWM_CHAN_A, 0); // 0%
    pwm_set_clkdiv(pwm, 25);
    pwm_set_enabled(pwm, 1);

    // I2C slave init
    gpio_init(PIN_I2C_SDA);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);

    gpio_init(PIN_I2C_SCL);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SCL);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);

    // I2C master init
    gpio_init(18); // SDA
    gpio_set_function(18, GPIO_FUNC_I2C);
    gpio_pull_up(18);

    gpio_init(19);
    gpio_set_function(19, GPIO_FUNC_I2C);
    gpio_pull_up(19);

    i2c_init(i2c1, I2C_BAUDRATE);

    // Initialise les interruptions sur la broche du signal A
    gpio_set_irq_enabled_with_callback(PIN_M1_SA, GPIO_IRQ_EDGE_RISE, true, interruptSignal1);
    irq_set_priority(IO_IRQ_BANK0, 0);

    M1motion.move(-50 * (PULSE_PER_TOUR / WHEEL_PERIMETER), signalM1.absStep);
}

void loop0()    {
    static bool beep = 0;
    static bool order_sent = false;
    static uint8_t order = FORCE_STOP;
    static uint32_t order_value = 0;
    
    if (i2c_SD.new_data)    { // Réception de l'ordre depuis l'i2c
        order_sent = false;
        order = i2c_SD.registers[0];
        order_value = i2c_SD.registers[1]<<24 | i2c_SD.registers[2]<<16 | i2c_SD.registers[3]<<8 | i2c_SD.registers[4];
        i2c_SD.new_data = false;
    }
    if (!order_sent) { // Tente d'envoyer l'ordre au core 1
        order_sent = talk(order,  order_value * (PULSE_PER_TOUR / WHEEL_PERIMETER));
        if (order_sent)  { // Si l'opération réussi on exécute l'ordre sur le core 0
            switch(order) {
            case MOVE:
                M1motion.move(order_value, signalM1.absStep);
                break;
            case ROTATE:
                M1motion.rotate(order_value, signalM1.absStep);
                break;
            case FORCE_STOP:
                M1motion.forceStop();
                break;
            }
        }
    }

    // === Comptage des fronts via les interruptions ===

    // /!\ vérifier que le timer ne soit pas déjà terminé
    // Attend la fin du temps de loop
    while (!(systick_hw->csr & 0x00010000));

    // === calculs des données ===

    // Enregistre et remet à zéro le compteur de pas
    signalM1.buff_index = !signalM1.buff_index;
    // 500 impulsion par tours & rapport du réducteur 1/10 = 5000 impulsions par tours
    signalM1.speed_rotation = (signalM1.buff_count[!signalM1.buff_index] / PULSE_PER_TOUR) * (1000. / LOOP_TIME);
    signalM1.buff_count[!signalM1.buff_index] = 0;

    // Calcul de la vitesse
    motor1.consigne = M1motion.getSpeedFromAdvancement(signalM1.absStep);
    // Asservissement vitesse
    routine(signalM1, motor1, 0);

    // === écriture de l'état des sorties ===

    //printf("s:%.3f ; pwm:%d ; sp:%.3f\n", signalM1.speed_rotation, motor1.pwm, motor1.speed);

    gpio_put(PIN_M1_DIR, motor1.direction);
    pwm_set_gpio_level(PIN_M1_PWM, motor1.pwm);

    // Permet de voir la fréquence d'exécution :
    beep = !beep;
    gpio_put(PIN_PROCO_0_FREQ, beep);
}

void launchCore1()  {
    multicore_launch_core1(core1);  // Lance le core1 (pour traitement signalM2 et du moteur 2)
    multicore_fifo_pop_blocking();  // Attend la fin de setup du core1
}