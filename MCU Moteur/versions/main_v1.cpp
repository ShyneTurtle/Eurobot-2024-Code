#include <pico/stdlib.h>
#include <stdio.h>
#include <hardware/gpio.h>
#include <pico/time.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <hardware/structs/systick.h>
#include <hardware/exception.h>

// Programme v1

#define FLAG_CORE1 1234   // Attend le setup du proco 1

// Moteur 1
#define SIGNAL_A 0          // Signal A mot 1
#define SIGNAL_B 2          // Signal B mot 1
#define CODE_FREQUENCE 10   // Utilisé pour suivre la fréquence d'exécution de la loop proco 0
#define LED_TEST 25
#define PWM 15              // PWM asservi pour le moteur 1
#define DRIVING_DIRECTION 4 // Indique le sens moteur 1

// Sens moteur 1 (gauche/droite)
volatile bool drivingDirection = 0;

// Mesure de temps
volatile uint32_t period = 0;     // periode du signal encodeur moteur 1
volatile uint64_t top = 0;        // Temps au dernier front montant moteur 1
volatile uint32_t timeOutTop = 0; //

// Utilisé pour augmenter la fiabilité de la mesure
#define BUFFER_SIZE 15
#define TIME_OUT 6667 // 6.7ms (on considère moteur ne tourne pas)

volatile uint32_t buffer[BUFFER_SIZE] = {0};
volatile int index = 0;
uint32_t median = 0;


// Timer : 
uint32_t counter64 = 0;
float clkFrequence = 0;    // Fréquence de la clock en Hz


// PID :
float pid[3] = { 4, 0.1, 0 };
int error = 0;
int deltaError = 0;
long sumError = 0;
int previousError = 0;
int consigne = 50; // 0 à 255

int speed = 0;


// === FONCTIONS ===

void InterruptSignalA(uint gpio, uint32_t events);

uint64_t GetTime64();

uint32_t Sort(uint32_t* tab);

int PID();

long constrain(long x, long min, long max);

int map(float x, float in_min, float in_max, float out_min, float out_max);

// === MAIN ===
int main()  {
  stdio_init_all();

  set_sys_clock_khz(100000, 0);   // clk_sys à 125MHz

  systick_hw->csr |= 0x00000005;  // Active le compteur de cycle avec l'horloge
  systick_hw->rvr = 0x00ffffff;    // Set la valeur max du compteur

  // LED
  gpio_init(LED_TEST);
  gpio_set_dir(LED_TEST, GPIO_OUT);

  // Sens moteur
  gpio_init(DRIVING_DIRECTION);
  gpio_set_dir(DRIVING_DIRECTION, GPIO_OUT);

  // Signal B
  gpio_init(SIGNAL_B);
  gpio_set_dir(SIGNAL_B, GPIO_IN);

  // Test exécution du code
  gpio_init(CODE_FREQUENCE);
  gpio_set_dir(CODE_FREQUENCE, GPIO_OUT);

  // PWM
  gpio_set_function(PWM, GPIO_FUNC_PWM);
  uint pwm = pwm_gpio_to_slice_num(PWM);
  pwm_set_wrap(pwm, 255); // Set max
  pwm_set_chan_level(pwm, PWM_CHAN_A, 0); // 0%
  pwm_set_clkdiv(pwm, 100);
  pwm_set_enabled(pwm, 1);
  
  // Initialise les interruptions sur la broche 0
  gpio_set_irq_enabled_with_callback(SIGNAL_A, GPIO_IRQ_EDGE_RISE, true, InterruptSignalA);
  irq_set_priority(IO_IRQ_BANK0, SIGNAL_A);


  // Test :
  gpio_put(LED_TEST, 1);
  bool beep = 0;

  // ================== Core0 loop ====================
  while(1)  {
    // Si on ne recoit pas de signal pendant plus de 6.7 ms on considère la vitesse du moteur à 0
    if (timer_hw->timelr - timeOutTop > TIME_OUT) {
      median = 0;
    }
    else {
      median = Sort((uint32_t*)buffer); // 150Hz to 141kHz <=> 850 000 to 850
      median = map(100000000.0/median, 0, 141000, 0, 255);
    }

    gpio_put(DRIVING_DIRECTION, drivingDirection);  // Indique le sens moteur
    
    speed = consigne - PID(); // Calcul de correction
    speed = constrain(speed, 0, 255);
    printf("s:%d / m:%ld\n", speed, median);
    
    pwm_set_gpio_level(PWM, speed);

    // Permet de voir la fréquence d'exécution :
    beep = !beep;
    gpio_put(CODE_FREQUENCE, beep);
  }
}

void InterruptSignalA(uint gpio, uint32_t events) {
  // On regarde le sens de la marche moteur en regardant le signal B
  drivingDirection = gpio_get(SIGNAL_B);

  // Si flag à 1 du compteur on fait une retenu
  if (systick_hw->csr & 0x00010000)   counter64++;
  
  // Enregistre la période
  buffer[index++] = GetTime64() - top;  
  top = GetTime64();

  // Time out (en cas de moteur arrêté)
  timeOutTop = timer_hw->timelr;

  if (index >= BUFFER_SIZE)  {
    index = 0;
  }
}

uint64_t GetTime64()  {
  return  (counter64<<24) + (0x00ffffff - systick_hw->cvr);
}

uint32_t Sort(uint32_t* tab) {
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
        // Swap
        buff = cpyTab[i+1];
        cpyTab[i+1] = cpyTab[i];
        cpyTab[i] = buff;
      }
    }
  }
  return cpyTab[BUFFER_SIZE/2];
}

int PID()  {
  error = median - consigne;
  
  deltaError = error - previousError;
  sumError += error;
  sumError = constrain(sumError, -5000, 5000);
  
  int correction = (int)(pid[0] * error + pid[1] * sumError + pid[2] * deltaError);
  previousError = error;
 
  return correction;
}

int map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

long constrain(long x, long min, long max) {
  if (x > max)      return max;
  else if (x < min) return min;
  return x;
}