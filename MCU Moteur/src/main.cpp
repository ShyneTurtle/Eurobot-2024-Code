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


// === DESCRIPTION ===
// Programme d'asservissement pour 2 moteurs en courbe de vitesse
// En cours de développement, non testé
// (2C : 2 moteurs, asservissement en Courbe de vitesse)

// === NOTES ===
// PWM potentiellement inversé
// Mises à jour sur les mesures de signaux et la génération du pwm
// Implémentation de l'I2C (esclave)

#define FLAG_CORE1  1234  // Flag de fin de setup du proco 1
#define FORWARD     1     // Marche avant
#define BACKWARD    0     // Marche arrière

// Moteur 1
#define PIN_M1_SA         3     // Pin du signal A, moteur 1
#define PIN_M1_SB         2     // Pin du signal B, moteur 1
#define PIN_PROCO_0_FREQ  10    // Pin indiquant la fréquence/2 du proco 0
#define PIN_M1_PWM        12    // Pin de commande PWM, moteur 1
#define PIN_M1_DIR        13    // Pin de direction moteur, moteur 1

// Moteur 2
#define PIN_M2_SA         1     // Pin du signal A moteur 2
#define PIN_M2_SB         0     // Pin du signal B moteur 2
#define PIN_PROCO_1_FREQ  11    // Pin indiquant la fréquence/2 du proco 1
#define PIN_M2_PWM        14    // Pin de commande PWM, moteur 2
#define PIN_M2_DIR        9     // Pin de direction moteur, moteur 2

// I2C
#define PIN_SDA           16
#define PIN_SCL           17
#define I2C_BAUD_RATE     100000  //100kHz
#define I2C_SLAVE_ADDRESS 0x17

// Interprétation des mesures
#define BUFFER_SIZE     50       // Taille du filtre median mobile appliqué sur les mesures
#define TIME_OUT        6667     // Si aucun signal n'est reçu après 6.7 ms, on considère le moteur à l'arrêt (µs)
#define CLOCK_FREQ_KHZ  100000   // Fréquence du rp2040 (kHz)
#define PWM_WRAP_VALUE  255     // Précision du pwm (1 -> 0xffff), influe aussi sur la fréquence du pwm

// Mesures de temps sur un encodeur et filtre median mobile
struct SignalProcessing  {
  volatile uint64_t top     = 0;      // Temps au dernier front montant du Signal de l'encodeur
  volatile bool     signalB = false;  // Niveau logique du signal B lors du déclenchement de l'interruption (indique le sens de rotation)
  volatile int64_t  absStep = 0;      // Nombre de pas depuis le lancement 

  volatile uint32_t buffer[BUFFER_SIZE];  // Tableau des dernières valeurs mesurées
  volatile int index  = 0;                // Indice sur la prochaine valeur remplacée sur le buffer
  int32_t lastResult  = 0;                // Dernier résultat du filtre (vitesse la plus probable) 0 => 65535
  bool updateMedian   = 0;                // Si =1, le calcul du filtre devrait être réalisé
};

// Structure pour un asservissement PID
struct PidController {
  float kp = 4;
  float ki = 0.2;
  float kd = 0;
  int32_t error = 0;            // Différence entre la consigne et la mesure
  int32_t deltaError = 0;       // Différence entre l'erreur précédente et l'erreur
  int32_t sumError = 0;         // Somme des erreurs
  int32_t previous_error = 0;   // Erreur précédente
};

// Commande et asservissement moteur
struct Motor  {
  int consigne = 0;             // Représente la vitesse voulu du moteur
  PidController pid;            // Structure vers des données d'asservissement
  int speed = 0;                // Valeur PWM calculé pour le moteur
  bool direction = FORWARD;     // Sens de rotation moteur
};

struct  {
  int32_t datas[256];
  uint8_t  mem_address = 0;
  bool master_writing = false;
} i2c_memory;

// Timer : 
volatile uint64_t counter64_0 = 0;     // Retenues pour une conversion 24 bits vers 64 bits, proco 0
volatile uint64_t counter64_1 = 0;     // Retenues pour une conversion 24 bits vers 64 bits, proco 1

SignalProcessing signalM1;
SignalProcessing signalM2;

// === FONCTIONS ===

void interruptSignal1(uint gpio, uint32_t events);  // Routine d'interruption sur le signal A moteur 1

void interruptSignal2(uint gpio, uint32_t events);  // Routine d'interruption du signal A, moteur 2

void routine(SignalProcessing& signal, Motor& motorControl, bool proco);  // Routine de loop identique aux deux proco

int32_t PID(PidController& pid);  // Calcul de correction PID

// === MAIN PROCO 0 ===
void setup() {
  // = INITIALISATION MATERIELLE =
  Serial.begin(115200);
  set_sys_clock_khz(CLOCK_FREQ_KHZ, 0);   // clk_sys à 100MHz

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
  pwm_set_clkdiv(pwm, 100);
  pwm_set_enabled(pwm, 1);

  // I2C init
  i2c_inst_t* i2c = i2c_default;
  i2c_init(i2c_default, 400000);
  gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(PIN_SCL);
  gpio_pull_up(PIN_SDA);
  
  // Initialise les interruptions sur la broche du signal A
  gpio_set_irq_enabled_with_callback(PIN_M1_SA, GPIO_IRQ_EDGE_RISE, true, interruptSignal1);
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
  gpio_init(PIN_M2_SB);
  gpio_set_dir(PIN_M2_SB, GPIO_IN);

  // Signal Dir
  gpio_init(PIN_M2_DIR);
  gpio_set_dir(PIN_M2_DIR, GPIO_OUT);

  // Test exécution du code
  gpio_init(PIN_PROCO_1_FREQ);
  gpio_set_dir(PIN_PROCO_1_FREQ, GPIO_OUT);

  // PWM
  gpio_set_function(PIN_M2_PWM, GPIO_FUNC_PWM);
  uint pwm = pwm_gpio_to_slice_num(PIN_M2_PWM);
  pwm_set_wrap(pwm, PWM_WRAP_VALUE); // Set max
  pwm_set_chan_level(pwm, PWM_CHAN_A, 0); // 0%
  pwm_set_clkdiv(pwm, 1);
  pwm_set_enabled(pwm, 1);
  
  // Initialise les interruptions sur la pin du signal A
  gpio_set_irq_enabled_with_callback(PIN_M2_SA, GPIO_IRQ_EDGE_RISE, true, interruptSignal2);
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

void routine(SignalProcessing& signal, Motor& motorControl, bool proco) {

    //Si le flag de dépassement du timer est à 1 on enregistre la retenue
    if (systick_hw->csr & 0x00010000)     {
      if (proco)  counter64_1++;
      else        counter64_0++;
    }

    // Si on ne recoit pas de signal pendant plus de 6.7 ms on considère la vitesse du moteur à 0
    int64_t test = getTime64(proco) - signal.top;
    if (test > (int64_t)((TIME_OUT * 0.001) * CLOCK_FREQ_KHZ)) { // (µs->ms) * clk_kHz
      signal.buffer[signal.index++] = 0;
      if (signal.index >= BUFFER_SIZE) signal.index = 0;
    }

    signal.lastResult = mean(signal.buffer); // 15µs (66667Hz) à v=0xffff ; 6.667ms (150Hz) à v=0 (TIME_OUT)
    if (signal.lastResult != 0) {
      uint32_t motorFrequency = (CLOCK_FREQ_KHZ * 1000.0)/signal.lastResult;
      motorFrequency = constrain(motorFrequency, 0, 66667);
      signal.lastResult = map(motorFrequency, 0, 66667, 0, PWM_WRAP_VALUE);
    }
    
    motorControl.pid.error = signal.lastResult - motorControl.consigne;
    motorControl.speed = motorControl.consigne - PID(motorControl.pid); // Calcul de correction

    //On tiens compte du sens de rotation moteur après la correction PID
    if (motorControl.speed < 0) {
      motorControl.speed *= -1;
      motorControl.direction = BACKWARD;
    }
    else  {
      motorControl.direction = FORWARD;
    }

    motorControl.speed = constrain(motorControl.speed, 0, PWM_WRAP_VALUE);
}

void interruptSignal1(uint gpio, uint32_t events) {
  // On regarde le sens de la marche moteur en regardant le signal B
  signalM1.signalB = gpio_get(PIN_M1_SB);

  // Si flag à 1 du compteur on fait une retenu
  if (systick_hw->csr & 0x00010000)   counter64_0++;
  
  // Enregistre la période
  signalM1.buffer[signalM1.index++] = getTime64(0) - signalM1.top;  
  signalM1.top = getTime64(0);

  // Ajoute un pas
  if (signalM1.signalB)     signalM1.absStep += 1;
  else                      signalM1.absStep -= 1;

  if (signalM1.index >= BUFFER_SIZE)  {
    signalM1.index = 0;
  }
}

void interruptSignal2(uint gpio, uint32_t events) {
  // On regarde le sens de la marche moteur en regardant le signal B
  signalM2.signalB = gpio_get(PIN_M2_SB);

  // Si flag à 1 du compteur on fait une retenu
  if (systick_hw->csr & 0x00010000)   counter64_1++;
  
  // Enregistre la période
  signalM2.buffer[signalM2.index++] = getTime64(1) - signalM2.top;  
  signalM2.top = getTime64(1);

  // Ajoute un pas
  if (signalM2.signalB)     signalM2.absStep += 1;
  else                      signalM2.absStep -= 1;

  if (signalM2.index >= BUFFER_SIZE)  {
    signalM2.index = 0;
  }
}

int32_t PID(PidController& pid)  {
  
  int32_t correction = 0;

  pid.deltaError = pid.error - pid.previous_error;
  pid.sumError  += pid.error;

  // On évite un trop gros dépassement. Ici, si l'erreur est la plus grande et avec un coeff i=1,
  // le moteur aura la consigne maximum en un seul cycle d'exécution
  pid.sumError = constrain(pid.sumError, - PWM_WRAP_VALUE * pid.ki, PWM_WRAP_VALUE * pid.ki);
  
  correction = (int32_t)(pid.kp * pid.error + pid.ki * pid.sumError + pid.kd * pid.deltaError);
  pid.previous_error = pid.error;
}