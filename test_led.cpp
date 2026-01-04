#include <pigpio.h>
#include <stdio.h>

int main(void)
{
    const int led_pin = 6; // GPIO à adapter

    if (gpioInitialise() < 0) {
        printf("Erreur: impossible d'initialiser pigpio\n");
        return 1;
    }

    gpioSetMode(led_pin, PI_OUTPUT);

    while (1) {
        gpioWrite(led_pin, 1);   // LED ON
        gpioDelay(500000);       // 500 ms

        gpioWrite(led_pin, 0);   // LED OFF
        gpioDelay(500000);       // 500 ms
    }

    // Jamais atteint ici, mais bonne pratique
    gpioTerminate();
    return 0;
}
