
#include "picoIncludes.hpp"
#include "platforme.hpp"
#include "vl53l5cx_api.hpp"
#include "vl53l5cx_buffers.hpp"
#include "dataProcessing.hpp"

#define PIN_SDA 4
#define PIN_SCL 5

#define PIN_BUTTON  2

//pull up de 10k sur l'I2C


int main()  {

  stdio_init_all();

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  gpio_init(PIN_BUTTON);
  gpio_set_dir(PIN_BUTTON, GPIO_IN);

  VL53L5CX_Configuration dev;
  VL53L5CX_ResultsData results;
  uint8_t rangingMode = VL53L5CX_RANGING_MODE_CONTINUOUS;
  uint8_t slt;
  dataProcessing dataP(&results, true);
  dataP.queryData();

  
  uint8_t status = 0;
  uint8_t ready = 0;
  uint32_t freq = 0;
  uint8_t resolution = 16;
  dataProcessing dataPro(&results, true);

  dev.platform.address = 0x29;

  // === I2C init ===
  i2c_inst_t* i2c = i2c_default;
  freq = i2c_init(i2c_default, 400000);
  gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(PIN_SCL);
  gpio_pull_up(PIN_SDA);

  dev.platform.i2c = i2c;

  sleep_ms(3500);

  printf("Setup begin...\n");
  printf("i2c speed : %ld\n", freq);

  // === VL53L5cx init ===

  //Test reconnaissance
  status = vl53l5cx_is_alive(&dev, &ready);
  if (status == 0)    printf("VL détecté sur l'adresse %x ; r:%d\n", dev.platform.address, ready);
  else            printf("VL non détecté, e:%d, r:%d\n", status, ready);

  //Init
  status = vl53l5cx_init(&dev); //le module doit être sous tension, LPn à 1
  if (status == 0)    printf("init OK\n");
  else            printf("init FAILED, e:%d\n", status);
  
  //Fréquence
  status = vl53l5cx_set_ranging_frequency_hz(&dev, 60);
  if (status == 0)    printf("set frequency OK\n");
  else            printf("set frequency FAILED, e:%d\n", status);

  //Range mode
  status = vl53l5cx_set_ranging_mode(&dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
  if (status == 0)    printf("set range OK\n");
  else            printf("set range FAILED, e:%d\n", status);

  //4x4
  status = vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_8X8);
  if (status == 0)    printf("set 4x4 OK\n");
  else            printf("set 4x4 FAILED, e:%d\n", status);

  printf("\nSetup done\n\n");
  printf("start ranging...\n");
  status = vl53l5cx_start_ranging(&dev);
  if (status == 0)  printf(" -->DONE\n");
  else          printf(" -->FAILED, e:%d\n", status);

  gpio_put(PICO_DEFAULT_LED_PIN, true);

  // === LOOP ===

  while (1) {
    
    if (gpio_get(PIN_BUTTON)) {

      status = vl53l5cx_check_data_ready(&dev, &ready);
      if ((!status) || ready)  {
        vl53l5cx_get_ranging_data(&dev, &results);
        printf("=======\n");
        dataPro.queryData();
      }
      else    printf("data not ready\n");
    }

    else    printf("waiting...\n");
    sleep_ms(300);
  }


  return 0;
}