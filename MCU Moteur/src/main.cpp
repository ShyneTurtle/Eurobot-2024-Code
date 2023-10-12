#include <Arduino.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <hardware/structs/systick.h>
#include <hardware/exception.h>
#include <hardware/irq.h>

// Programme en distance

#define FLAG_CORE1  1234  // Flag de fin de setup du proco 1
#define FORWARD     1     // Marche avant
#define BACKWARD    0     // Marche arrière

// Moteur 1
#define PIN_SM1_A         0     // Signal A moteur 1
#define PIN_SM1_B         2     // Signal B moteur 1
#define PIN_PROCO_0_FREQ  10    // Pin indiquant la fréquence du proco 0
#define PIN_PWM_M1        12    // PWM asservi pour le moteur 1(enable)
#define PIN_DIR_M1        14    // Direction moteur 1

// Moteur 2
#define PIN_SM2_A         1      // Signal A moteur 2
#define PIN_SM2_B         3      // Signal B moteur 2
#define PIN_PROCO_1_FREQ  11     // Pin indiquant la fréquence du proco 1
#define PIN_PWM_M2        13     // PWM asservi pour le moteur 2
#define PIN_DIR_M2        15     // Direction moteur 2

// Interprétation des mesures
#define CLOCK_FREQ_KHZ  100000   // Fréquence du rp2040 (kHz)
#define PWM_WRAP_VALUE  0xffff   // précision du pwm (0 -> 0xffff)

// Structure pour un asservissement PID
struct PidController {
  float p = 1;
  float i = 0.01;
  float d = 0;
  int  error = 0;           // Différence entre la step_target et la mesure
  int  delta_error = 0;      // Différence entre l'erreur précédente et l'erreur
  long sum_error = 0;        // Somme des erreurs
  int  previous_error = 0;   // Erreur précédente
};

// Commande et asservissement moteur
struct Motor {
  volatile int64_t step_target = 0;         // Nombre de pas restant
  PidController pid;            // Asservissement PID du moteur
  uint16_t speed = 0;
  bool dir = FORWARD;
  volatile int64_t abs_foot = 0; // Nombre de pas par rapport à la position de départ
};

Motor motor1;
Motor motor2;

// === FONCTIONS ===

void interruptSignal1(uint gpio, uint32_t events);  // Routine d'interruption sur le signal A moteur 1

void interruptSignal2(uint gpio, uint32_t events);  // Routine d'interruption sur le signal A moteur 2

void routine(Motor& motor_control, bool proco);  // Routine de loop identique aux deux proco

void pid(PidController& pid, bool& dir, uint16_t& speed);  // Correction PID

// === MAIN PROCO 0 ===
void setup() {
  // = INITIALISATION MATERIELLE =
  Serial.begin(115200);
  set_sys_clock_khz(CLOCK_FREQ_KHZ, 0);   // clk_sys à 100MHz

  // Signal B
  gpio_init(PIN_SM1_B);
  gpio_set_dir(PIN_SM1_B, GPIO_IN);

  // Signal Dir
  gpio_init(PIN_DIR_M1);
  gpio_set_dir(PIN_DIR_M1, GPIO_OUT);

  // Fréquence d'exécution du code
  gpio_init(PIN_PROCO_0_FREQ);
  gpio_set_dir(PIN_PROCO_0_FREQ, GPIO_OUT);

  // PWM
  gpio_set_function(PIN_PWM_M1, GPIO_FUNC_PWM);
  uint pwm = pwm_gpio_to_slice_num(PIN_PWM_M1);
  pwm_set_wrap(pwm, PWM_WRAP_VALUE); // Set max
  pwm_set_chan_level(pwm, PWM_CHAN_A, 0); // 0%
  pwm_set_clkdiv(pwm, 100);
  pwm_set_enabled(pwm, 1);

  // Initialise les interruptions sur la broche 0
  gpio_set_irq_enabled_with_callback(PIN_SM1_A, GPIO_IRQ_EDGE_RISE, true, interruptSignal1);
  irq_set_priority(IO_IRQ_BANK0, 0);

  motor1.step_target = -300000;   // Avancer de x pas
}
bool beep0 = 0; // Beep Beep (debug execution des procos)
void loop() {
  routine(motor1, 0);

  // Serial.printf("v:%ld ; a:%ld ; c:%ld ; sum: %ld ; +s:%ld\n", motor1.speed, (int32_t)motor1.abs_foot, (int32_t)motor1.step_target, motor1.pid.sum_error, (uint32_t)(motor1.pid.sum_error * motor1.pid.i));
  // Serial.printf("v:%ld\n", motor1.speed);

  gpio_put(PIN_DIR_M1, !motor1.dir);
  pwm_set_gpio_level(PIN_PWM_M1, motor1.speed);

  // Permet de voir la fréquence d'exécution :
  beep0 = !beep0;
  gpio_put(PIN_PROCO_0_FREQ, beep0);
}

void setup1() {
  // Signal B
  gpio_init(PIN_SM2_B);
  gpio_set_dir(PIN_SM2_B, GPIO_IN);

  // Signal Dir
  gpio_init(PIN_DIR_M2);
  gpio_set_dir(PIN_DIR_M2, GPIO_OUT);

  // Test exécution du code
  gpio_init(PIN_PROCO_1_FREQ);
  gpio_set_dir(PIN_PROCO_1_FREQ, GPIO_OUT);

  // PWM
  gpio_set_function(PIN_PWM_M2, GPIO_FUNC_PWM);
  uint pwm = pwm_gpio_to_slice_num(PIN_PWM_M2);
  pwm_set_wrap(pwm, PWM_WRAP_VALUE); // Set max (théoriquement plus de précision, le max étant 65535)
  pwm_set_chan_level(pwm, PWM_CHAN_A, 0); // 0%
  pwm_set_clkdiv(pwm, 100);
  pwm_set_enabled(pwm, 1);

  // Initialise les interruptions sur la broche 0
  gpio_set_irq_enabled_with_callback(PIN_SM2_A, GPIO_IRQ_EDGE_RISE, true, interruptSignal2);
  irq_set_priority(IO_IRQ_BANK0, 0);

  // Fin de setup
  multicore_fifo_push_blocking(FLAG_CORE1);


  motor2.step_target = 2000;
  motor2.dir = FORWARD;
}
bool beep1 = 0;
void loop1() {
  routine(motor2, 1);

  // Serial.printf("s:%d / m:%ld\n", motor2.speed, signalM2.lastResult);

  gpio_put(PIN_DIR_M2, !motor2.dir);
  pwm_set_gpio_level(PIN_PWM_M2, motor2.speed);

  // Permet de voir la fréquence d'exécution :
  beep1 = !beep1;
  gpio_put(PIN_PROCO_1_FREQ, beep1);
}

void routine(Motor& motor_control, bool proco) {
  motor_control.pid.error = motor_control.step_target;     // Nombre de pas restant

  pid(motor_control.pid, motor_control.dir, motor_control.speed);
  motor_control.speed = constrain(motor_control.speed, 0, PWM_WRAP_VALUE);
}

void interruptSignal1(uint gpio, uint32_t events) {
  // On obtient le sens de la marche moteur en regardant le signal B
  if (gpio_get(PIN_SM1_B)) {
    motor1.abs_foot += 1;
    motor1.step_target -= 1;
  } else {
    motor1.abs_foot -= 1;
    motor1.step_target += 1;
  }
}

void interruptSignal2(uint gpio, uint32_t events) {
  // On obtient le sens de la marche moteur en regardant le signal B
  if (gpio_get(PIN_SM2_B)) {
    motor2.abs_foot += 1;
    motor2.step_target -= 1;
  } else {
    motor2.abs_foot -= 1;
    motor2.step_target += 1;
  }
}

void pid(PidController& pid, bool& dir, uint16_t& speed) {
  static int32_t correction = 0;

  pid.delta_error = pid.error - pid.previous_error;
  pid.sum_error += pid.error / 30;

  // On évite un trop gros dépassement. Ici, si l'erreur est la plus grande et avec un coeff i=1,
  // le moteur aura la step_target maximum en un seul cycle d'exécution
  pid.sum_error = constrain(pid.sum_error, -PWM_WRAP_VALUE, PWM_WRAP_VALUE);

  correction = (int32_t)(pid.p * pid.error + pid.i * pid.sum_error + pid.d * pid.delta_error);
  correction = constrain(correction, -PWM_WRAP_VALUE, PWM_WRAP_VALUE);

  // Applique la vitesse et la direction des moteurs
  if (correction >= 0) {
    dir = FORWARD;
    speed = correction;
  } else {
    dir = BACKWARD;
    speed = correction * -1;
  }

  pid.previous_error = pid.error;
}