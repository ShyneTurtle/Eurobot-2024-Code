//Programme de test

#include <pico/stdlib.h>
#include <stdio.h>

int main()  {

    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    while(1) {
        sleep_ms(100);
        gpio_put(25, 1);
        sleep_ms(100);
        gpio_put(25, 0);
    }    
    return 0;
}