#include <libpynq.h>
#include <stdio.h>

int main(void){
    pynq_init();
    
    gpio_init();


    switchbox_set_pin(IO_A0, SWB_GPIO);
    switchbox_set_pin(IO_A1, SWB_GPIO);
    
    gpio_set_direction(IO_A0, GPIO_DIR_INPUT);
    gpio_set_direction(IO_A1, GPIO_DIR_INPUT);

    while(1){
        gpio_level_t c0 = gpio_get_level(IO_A0);
        gpio_level_t c1 = gpio_get_level(IO_A1);
        printf("Message 0: %d\n", c0);
        printf("Message 1: %d\n", c1);
        sleep_msec(25);
    }

    gpio_reset();
    gpio_destroy();
    
    pynq_destroy();

    return EXIT_SUCCESS;
}
