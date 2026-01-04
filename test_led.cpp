#include <pigpiod_if2.h>
#include <stdio.h>

int main() {
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) { printf("Connexion pigpiod impossible\n"); return 1; }

    int led_pin = 6;
    set_mode(pi, led_pin, PI_OUTPUT);

    while (1) {
        gpio_write(pi, led_pin, 1); time_sleep(0.5);
        gpio_write(pi, led_pin, 0); time_sleep(0.5);
    }

    pigpio_stop(pi);
    return 0;
}
