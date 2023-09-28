#include <pico/stdlib.h>
#include <stdio.h>
#include <hardware/gpio.h>
#include <pico/time.h>
#include <pico/multicore.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <hardware/structs/systick.h>
#include <hardware/exception.h>

//Programme en distance

#define FLAG_CORE1  1234  //Flag de fin de setup du proco 1
#define FORWARD     1     //Marche avant
#define BACKWARD    0     //Marche arrière

//Moteur 1
#define PIN_SM1_A         0     //Signal A moteur 1
#define PIN_SM1_B         2     //Signal B moteur 1
#define PIN_PROCO_0_FREQ  10    //Pin indiquant la fréquence du proco 0
#define PIN_PWM_M1        12    //PWM asservi pour le moteur 1(enable)
#define PIN_DIR_M1        14    //Direction moteur 1

//Moteur 2
#define PIN_SM2_A         1      //Signal A moteur 2
#define PIN_SM2_B         3      //Signal B moteur 2
#define PIN_PROCO_1_FREQ  11     //Pin indiquant la fréquence du proco 1
#define PIN_PWM_M2        13     //PWM asservi pour le moteur 2
#define PIN_DIR_M2        15     //Direction moteur 2

//Interprétation des mesures
#define CLOCK_FREQ_KHZ  100000   //Fréquence du rp2040 (kHz)
#define PWM_WRAP_VALUE  0xffff   //précision du pwm (0 -> 0xffff)

//Structure pour un asservissement PID
struct pidController {
  float p = 1;
  float i = 0.01;
  float d = 0;
  int  error = 0;           //Différence entre la consigne et la mesure
  int  deltaError = 0;      //Différence entre l'erreur précédente et l'erreur
  long sumError = 0;        //Somme des erreurs
  int  previousError = 0;   //Erreur précédente
};

//Commande et asservissement moteur
struct motor  {
  volatile int64_t consigne = 0;         //Nombre de pas restant
  pidController pid;            //Asservissement PID du moteur
  uint16_t speed = 0;
  bool dir = FORWARD;
  volatile int64_t absFoot = 0; //Nombre de pas par rapport à la position de départ
};

motor motor1;
motor motor2;

//=== FONCTIONS ===

void Core1(); //Programme exécuté sur le proco 1

void InterruptSignal1(uint gpio, uint32_t events);  //Routine d'interruption sur le signal A moteur 1

void InterruptSignal2(uint gpio, uint32_t events);  //Routine d'interruption sur le signal A moteur 2

void Routine(motor& motorControl, bool proco);  //Routine de loop identique aux deux proco

void PID(pidController& pid, bool &dir, uint16_t &speed);  //Correction PID

long constrain(long x, long min, long max);

//=== MAIN PROCO 0 ===
int main()  {

  //= INITIALISATION MATERIELLE =
  stdio_init_all();
  set_sys_clock_khz(CLOCK_FREQ_KHZ, 0);   //clk_sys à 100MHz

  //Signal B
  gpio_init(PIN_SM1_B);
  gpio_set_dir(PIN_SM1_B, GPIO_IN);

  //Signal Dir
  gpio_init(PIN_DIR_M1);
  gpio_set_dir(PIN_DIR_M1, GPIO_OUT);

  //Fréquence d'exécution du code
  gpio_init(PIN_PROCO_0_FREQ);
  gpio_set_dir(PIN_PROCO_0_FREQ, GPIO_OUT);

  //PWM
  gpio_set_function(PIN_PWM_M1, GPIO_FUNC_PWM);
  uint pwm = pwm_gpio_to_slice_num(PIN_PWM_M1);
  pwm_set_wrap(pwm, PWM_WRAP_VALUE); //Set max
  pwm_set_chan_level(pwm, PWM_CHAN_A, 0); //0%
  pwm_set_clkdiv(pwm, 100);
  pwm_set_enabled(pwm, 1);
  
  //Initialise les interruptions sur la broche 0
  gpio_set_irq_enabled_with_callback(PIN_SM1_A, GPIO_IRQ_EDGE_RISE, true, InterruptSignal1);
  irq_set_priority(IO_IRQ_BANK0, 0);

  multicore_launch_core1(Core1);  //Lance le core1 (pour traitement signalM2 et du moteur 2)
  multicore_fifo_pop_blocking();  //Attend la fin de setup du core1


  bool beep = 0;
  motor1.consigne = -300000;   //Avancer de x pas

  //================== proco_0 loop ====================
  while(1)  {
    
    Routine(motor1, 0);

    //printf("v:%ld ; a:%ld ; c:%ld ; sum: %ld ; +s:%ld\n", motor1.speed, (int32_t)motor1.absFoot, (int32_t)motor1.consigne, motor1.pid.sumError, (uint32_t)(motor1.pid.sumError * motor1.pid.i));
    //printf("v:%ld\n", motor1.speed);

    gpio_put(PIN_DIR_M1, !motor1.dir);
    pwm_set_gpio_level(PIN_PWM_M1, motor1.speed);

    //Permet de voir la fréquence d'exécution :
    beep = !beep;
    gpio_put(PIN_PROCO_0_FREQ, beep);
  }
}

void Core1()  {

  //Signal B
  gpio_init(PIN_SM2_B);
  gpio_set_dir(PIN_SM2_B, GPIO_IN);

  //Signal Dir
  gpio_init(PIN_DIR_M2);
  gpio_set_dir(PIN_DIR_M2, GPIO_OUT);

  //Test exécution du code
  gpio_init(PIN_PROCO_1_FREQ);
  gpio_set_dir(PIN_PROCO_1_FREQ, GPIO_OUT);

  //PWM
  gpio_set_function(PIN_PWM_M2, GPIO_FUNC_PWM);
  uint pwm = pwm_gpio_to_slice_num(PIN_PWM_M2);
  pwm_set_wrap(pwm, PWM_WRAP_VALUE); //Set max (théoriquement plus de précision, le max étant 65535)
  pwm_set_chan_level(pwm, PWM_CHAN_A, 0); //0%
  pwm_set_clkdiv(pwm, 100);
  pwm_set_enabled(pwm, 1);
  
  //Initialise les interruptions sur la broche 0
  gpio_set_irq_enabled_with_callback(PIN_SM2_A, GPIO_IRQ_EDGE_RISE, true, InterruptSignal2);
  irq_set_priority(IO_IRQ_BANK0, 0);

  //Fin de setup
  multicore_fifo_push_blocking(FLAG_CORE1);

  bool beep;

  motor2.consigne = 2000;
  motor2.dir = FORWARD;

  // ============= proco_1 loop =============
  while(true) { 
    
    Routine(motor2, 1);

    //printf("s:%d / m:%ld\n", motor2.speed, signalM2.lastResult);
    
    gpio_put(PIN_DIR_M2, !motor2.dir);
    pwm_set_gpio_level(PIN_PWM_M2, motor2.speed);

    //Permet de voir la fréquence d'exécution :
    beep = !beep;
    gpio_put(PIN_PROCO_1_FREQ, beep);
  }
}

void Routine(motor& motorControl, bool proco) {
    
    motorControl.pid.error = motorControl.consigne;     //Nombre de pas restant

    PID(motorControl.pid, motorControl.dir, motorControl.speed);
    motorControl.speed = constrain(motorControl.speed, 0, PWM_WRAP_VALUE);
}

void InterruptSignal1(uint gpio, uint32_t events) {
  //On regarde le sens de la marche moteur en regardant le signal B
  if (gpio_get(PIN_SM1_B))  {
    motor1.absFoot += 1;
    motor1.consigne -= 1;
  }
  else  {
    motor1.absFoot -= 1;
    motor1.consigne += 1;
  }
}

void InterruptSignal2(uint gpio, uint32_t events) {
  //On regarde le sens de la marche moteur en regardant le signal B
  if (gpio_get(PIN_SM2_B))  {
    motor2.absFoot += 1;
    motor2.consigne -= 1;
  }
  else  {
    motor2.absFoot -= 1;
    motor2.consigne += 1;
  }
}

void PID(pidController& pid, bool &dir, uint16_t &speed)  {
  
  static int32_t correction = 0;

  pid.deltaError = pid.error - pid.previousError;
  pid.sumError  += pid.error / 30;

  //On évite un trop gros dépassement. Ici, si l'erreur est la plus grande et avec un coeff i=1,
  //le moteur aura la consigne maximum en un seul cycle d'exécution
  pid.sumError   = constrain(pid.sumError, - PWM_WRAP_VALUE, PWM_WRAP_VALUE);
  
  correction = (int32_t)(pid.p * pid.error + pid.i * pid.sumError + pid.d * pid.deltaError);
  correction = constrain(correction, - PWM_WRAP_VALUE, PWM_WRAP_VALUE);

  //Applique la vitesse et la direction des moteurs
  if (correction >= 0)    {
    dir = FORWARD;
    speed = correction;
  }
  else {
    dir = BACKWARD;
    speed = correction * -1;
  }

  pid.previousError = pid.error;
}

long constrain(long x, long min, long max) {
  if (x > max)      return max;
  else if (x < min) return min;
  return x;
}