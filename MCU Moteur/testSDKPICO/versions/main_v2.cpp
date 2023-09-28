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

//Programme en courbe de vitesse
//PWM potentiellement inversé

#define FLAG_CORE1  1234  //Flag de fin de setup du proco 1
#define FORWARD     1     //Marche avant
#define BACKWARD    0     //Marche arrière

//Moteur 1
#define PIN_SM1_A         3     //Signal A moteur 1
#define PIN_SM1_B         2     //Signal B moteur 1
#define PIN_PROCO_0_FREQ  10    //Pin indiquant la fréquence du proco 0
#define PIN_PWM_M1        12    //PWM asservi pour le moteur 1
#define PIN_DIR_M1        13    //Direction moteur

//Moteur 2
#define PIN_SM2_A         1      //Signal A moteur 2
#define PIN_SM2_B         0      //Signal B moteur 2
#define PIN_PROCO_1_FREQ  11     //Pin indiquant la fréquence du proco 1
#define PIN_PWM_M2        14     //PWM asservi pour le moteur 2
#define PIN_DIR_M2        15    //Direction moteur

//Interprétation des mesures
#define BUFFER_SIZE     15       //Taille du filtre median mobile (appliqué sur les mesures)
#define TIME_OUT        6667     //Si aucun signal n'est reçu après 6.7 ms, on considère le moteur à l'arrêt
#define CLOCK_FREQ_KHZ  100000   //Fréquence du rp2040 (kHz)
#define PWM_WRAP_VALUE  0xffff   //précision du pwm (0 -> 0xffff)

//Mesures de temps sur un encodeur et filtre median mobile
struct signalProcessing  {
  volatile uint32_t period  = 0;      //Periode mesuré du Signal de l'encodeur
  volatile uint64_t top     = 0;      //Temps au dernier front montant du Signal de l'encodeur
  volatile bool     signalB = false;  //Niveau logique du signal B lors du déclenchement de l'interruption (indique le sens de rotation)
  volatile int64_t  absFoot = 0;      //Nombre de pas depuis le lancement 

  volatile uint32_t buffer[BUFFER_SIZE];  //Tableau des dernières valeurs mesurées
  volatile int index  = 0;                //Indice sur la prochaine valeur remplacée sur le buffer
  uint32_t lastResult = 0;                //Dernier résultat du filtre (vitesse la plus probable) 0 => 65535
  bool updateMedian   = 0;                //Si =1, le calcul du filtre devrait être réalisé
};

//Structure pour un asservissement PID
struct pidController {
  float p = 0;
  float i = 0;
  float d = 0;
  int  error = 0;           //Différence entre la consigne et la mesure
  int  deltaError = 0;      //Différence entre l'erreur précédente et l'erreur
  long sumError = 0;        //Somme des erreurs
  int  previousError = 0;   //Erreur précédente
};

//Commande et asservissement moteur
struct motor  {
  int consigne = 0;             //Représente la vitesse voulu du moteur
  pidController pid;            //Asservissement PID du moteur
  int speed = 0;                //Valeur PWM donnée au moteur
  bool direction = FORWARD;     //Sens de rotation
};

//Timer : 
uint64_t counter64_0 = 0;     //Retenues pour une conversion 24 bits vers 64 bits, proco 0
uint64_t counter64_1 = 0;     //Retenues pour une conversion 24 bits vers 64 bits, proco 1

signalProcessing signalM1;
signalProcessing signalM2;

//=== FONCTIONS ===

void Core1(); //Programme exécuté sur le proco 1

void InterruptSignal1(uint gpio, uint32_t events);  //Routine d'interruption sur le signal A moteur 1

void InterruptSignal2(uint gpio, uint32_t events);  //Routine d'interruption sur le signal A moteur 2

void Routine(signalProcessing& signal, motor& motorControl);  //Routine de loop identique aux deux proco

int PID(pidController& pid);  //Correction PID

uint32_t Sort(volatile uint32_t* tab);   //Calcul du filtre

inline uint64_t GetTime64(bool proco);   //Conversion du temps sur 64 bits (fréquence liée à la fréquence des procos)

long constrain(long x, long min, long max);

uint16_t map(float x, float in_min, float in_max, float out_min, float out_max);

//=== MAIN PROCO 0 ===
int main()  {

  //= INITIALISATION MATERIELLE =
  stdio_init_all();
  set_sys_clock_khz(CLOCK_FREQ_KHZ, 0);   //clk_sys à 100MHz

  //Gestion du temps : compteur de cycles
  systick_hw->csr |= 0x00000005;    //Active le compteur de cycle avec l'horloge
  systick_hw->rvr  = 0x00ffffff;    //Set la valeur max du compteur

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
  motor motor1;

  motor1.consigne = 0xffff * 0.5;
  motor1.direction = FORWARD;

  //================== proco_0 loop ====================
  while(1)  {
    
    Routine(signalM1, motor1, 0);

    printf("s:%d / m:%ld\n", motor1.speed, signalM1.lastResult);
    
    gpio_put(PIN_DIR_M1, motor1.direction);
    pwm_set_gpio_level(PIN_PWM_M1, motor1.speed);

    //Permet de voir la fréquence d'exécution :
    beep = !beep;
    gpio_put(PIN_PROCO_0_FREQ, beep);
  }
}

void Core1()  {

  //Gestion du temps : compteur de cycles (unique à chaque proco)
  systick_hw->csr |= 0x00000005;    //Active le compteur de cycle avec l'horloge
  systick_hw->rvr  = 0x00ffffff;    //Set la valeur max du compteur

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
  motor motor2;

  motor2.consigne = 0xffff * 0.5;
  motor2.direction = FORWARD;

  // ============= proco_1 loop =============
  while(true) { 
    
    Routine(signalM2, motor2, 1);

    //printf("s:%d / m:%ld\n", motor2.speed, signalM2.lastResult);
    
    gpio_put(PIN_DIR_M2, motor2.direction);
    pwm_set_gpio_level(PIN_PWM_M2, motor2.speed);

    //Permet de voir la fréquence d'exécution :
    beep = !beep;
    gpio_put(PIN_PROCO_1_FREQ, beep);
  }
}

void Routine(signalProcessing& signal, motor& motorControl, bool proco) {

    //Si on ne recoit pas de signal pendant plus de 6.7 ms on considère la vitesse du moteur à 0
    if (GetTime64(proco) - signal.top > TIME_OUT) {
      signal.lastResult = 0;
    }
    else if (signal.updateMedian) {
      signal.lastResult = Sort(signal.buffer); //150Hz to 141kHz <=> 850 000 to 850
      signal.lastResult = map((CLOCK_FREQ_KHZ * 1000.0)/signal.lastResult, 0, 141000, 0, PWM_WRAP_VALUE);
      signal.updateMedian = false;
    }
    
    motorControl.pid.error = signal.lastResult - motorControl.consigne;//* (-1 * signalM2.signalB)

    motorControl.speed = motorControl.consigne - PID(motorControl.pid); //Calcul de correction (consigne - correction)
    motorControl.speed = constrain(motorControl.speed, 0, PWM_WRAP_VALUE);

    if (motorControl.speed < 0)   motorControl.direction = BACKWARD;
    else                          motorControl.direction = FORWARD;
}

void InterruptSignal1(uint gpio, uint32_t events) {
  //On regarde le sens de la marche moteur en regardant le signal B
  signalM1.signalB = gpio_get(PIN_SM1_B);

  //Si flag à 1 du compteur on fait une retenu
  if (systick_hw->csr & 0x00010000)   counter64_0++;
  
  //Enregistre la période
  signalM1.buffer[signalM1.index++] = GetTime64(0) - signalM1.top;  
  signalM1.top = GetTime64(0);

  //Ajoute un pas
  if (signalM1.signalB)     signalM1.absFoot += 1;
  else                      signalM1.absFoot -= 1;

  if (signalM1.index >= BUFFER_SIZE)  {
    signalM1.index = 0;
  }
}

void InterruptSignal2(uint gpio, uint32_t events) {
  //On regarde le sens de la marche moteur en regardant le signal B
  signalM2.signalB = gpio_get(PIN_SM2_B);

  //Si flag à 1 du compteur on fait une retenu
  if (systick_hw->csr & 0x00010000)   counter64_1++;
  
  //Enregistre la période
  signalM2.buffer[signalM2.index++] = GetTime64(1) - signalM2.top;  
  signalM2.top = GetTime64(1);

  if (signalM2.index >= BUFFER_SIZE)  {
    signalM2.index = 0;
  }
}

int PID(pidController& pid)  {
  
  static int correction = 0;

  pid.deltaError = pid.error - pid.previousError;
  pid.sumError  += pid.error;

  //On évite un trop gros dépassement. Ici, si l'erreur est la plus grande et avec un coeff i=1,
  //le moteur aura la consigne maximum en un seul cycle d'exécution
  pid.sumError   = constrain(pid.sumError, - PWM_WRAP_VALUE * pid.i, PWM_WRAP_VALUE * pid.i);
  
  correction = (int)(pid.p * pid.error + pid.i * pid.sumError + pid.d * pid.deltaError);
  pid.previousError = pid.error;
 
  return correction;
}

inline uint64_t GetTime64(bool proco)  {
  if      (proco == 0)    return (counter64_0<<24) + (0x00ffffff - systick_hw->cvr);
  else if (proco == 1)    return (counter64_1<<24) + (0x00ffffff - systick_hw->cvr);
}

uint32_t Sort(volatile uint32_t* tab) {
  bool sort = 1;
  uint32_t buff = 0;
  uint32_t cpyTab[BUFFER_SIZE] = {0};
  for (int i = 0; i<BUFFER_SIZE; i++)  {
    cpyTab[i] = tab[i];
  }
  while (sort)  {
    sort = 0;
    for (int i = 0; i<BUFFER_SIZE - 1; i++) {
      if (cpyTab[i] > cpyTab[i+1])  {
        sort = 1;
        //Swap
        buff = cpyTab[i+1];
        cpyTab[i+1] = cpyTab[i];
        cpyTab[i] = buff;
      }
    }
  }
  return cpyTab[BUFFER_SIZE/2];
}

uint16_t map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (uint16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

long constrain(long x, long min, long max) {
  if (x > max)      return max;
  else if (x < min) return min;
  return x;
}