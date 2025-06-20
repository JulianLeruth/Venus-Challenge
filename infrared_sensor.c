#include <libpynq.h>
#include <stdio.h>
#include <switchbox.h>

#include "infrared_sensor.h"

int IRborderDetection() {
    gpio_set_direction(IO_AR5, GPIO_DIR_INPUT);
    int rightside = gpio_get_level(IO_AR5);
    // printf("Right side: %d\n", rightside);

    gpio_set_direction(IO_AR6, GPIO_DIR_INPUT);
    int leftside = gpio_get_level(IO_AR6);
    // printf("Left side: %d\n", leftside);

    if (rightside == 1 && leftside == 1) return IR_BOTH;
    else if (rightside == 1) return IR_RIGHT;
    else if (leftside == 1) return IR_LEFT;
    else return IR_NONE;
}
