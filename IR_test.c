#include <libpynq.h>
#include <stdio.h>

int main(void){
    pynq_init();
    
    gpio_init();


    switchbox_set_pin(IO_A0, SWB_GPIO);

    gpio_set_direction(IO_A0, GPIO_DIR_INPUT);

    while(1){
        gpio_level_t c = gpio_get_level(IO_A0);
        printf("Message: %d\n", c);
        sleep_msec(25);
    }

    gpio_reset();
    gpio_destroy();
    
    pynq_destroy();

    return EXIT_SUCCESS;
}
