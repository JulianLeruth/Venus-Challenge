#include <libpynq.h>
#include <stdio.h>
#include <switchbox.h>

int IRborderDetection() {
    gpio_init();


    gpio_set_direction(IO_AR4, GPIO_DIR_INPUT);
    int rightside = gpio_get_level(IO_AR4);
    printf("Right side: %d\n", rightside);


    gpio_set_direction(IO_AR5, GPIO_DIR_INPUT);
    int leftside = gpio_get_level(IO_AR5);
    printf("Left side: %d\n", leftside);


    sleep_msec(25);

    if (rightside > 0 && leftside > 0) return 2;
    else if (rightside > 0) return 1;
    else if (leftside > 0) return 0;
    else return -1;
}
