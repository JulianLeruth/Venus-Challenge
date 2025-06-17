#include <libpynq.h>
#include <switchbox.h>
#include <stepper.h>
#include <stdio.h>
#include <math.h>
#include <iic.h>
#include <gpio.h>

#include "vl53l0x.h"
#include "tcs3472.h"
#include "tca9548a.h"

#include "movement_lib.h"
#include "distance_sensor.h"
#include "colour_sensor.h"
#include "infrared_sensor.h"

struct block_t {
    int size;
    int color;
    int x_loc, y_loc;
};

void __destroy__(tca9548a* mux, int succes) {
    if (succes == EXIT_SUCCESS) printf("\033[32m✔ All operations completed successfully. \033[0mShutting down and cleaning up resources...\n");
    else if (succes == EXIT_FAILURE) printf("\033[31m✘ Error found, all operations stopped. \033[0mShutting down and cleaning up resources...\n");
    if (mux != NULL) tca9548a_destroy(mux);
    iic_destroy(IIC0);
    stepper_destroy();
    pynq_destroy();
}

int main(void) {
    /* =========================== Initization ============================ */
    pynq_init();
    iic_init(IIC0);
    stepper_init();
    
    double r_loc_x = 50;            // The X location of the robot, relative to the starting position.
    double r_loc_y = 50;            // The Y location of the robot, relative to the starting position.
    double r_angle = 0;             // The angle of the robot, relative to the starting position.
    
    int function;                   // A variable for checking function outputs.
    /* ========================= Switchbox Set-up ========================= */
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    
    switchbox_set_pin(IO_AR5, SWB_GPIO);
    switchbox_set_pin(IO_AR6, SWB_GPIO);
    
    switchbox_set_pin(IO_AR4, SWB_GPIO);
    gpio_set_direction(IO_AR4, GPIO_DIR_OUTPUT);
    gpio_set_level(IO_AR4, GPIO_LEVEL_LOW);
    
    /* ======================== Multiplexer Set-up ======================== */
    tca9548a mux;
    if (tca9548a_init(IIC0, &mux) == EXIT_FAILURE) {
        fprintf(stderr, "\033[31m✘ Failed to initialize TCA9548A multiplexer\n\n\033[0m");
        iic_destroy(IIC0);
        return EXIT_FAILURE;
    }

    /* ====================== Distance Sensor Set-up ====================== */
    vl53x dist_sensor;
	if (tca9548a_switch_channel(&mux, MUX_CHANNEL_DIST_SENSOR_0) == EXIT_FAILURE) return EXIT_FAILURE;
    printf("\033[35m✔ Channel %d selected\033[0m — ready to talk to VL53L0X distance sensor on channel %d.\n", MUX_CHANNEL_DIST_SENSOR_0, MUX_CHANNEL_DIST_SENSOR_0);
    if (vl53l0xPing(&dist_sensor) == EXIT_FAILURE) {
        fprintf(stderr, "\033[31m✘ Failed to initialize VL53L0X distance sensor\n\n\033[0m");
        __destroy__(&mux, EXIT_FAILURE);
        return EXIT_FAILURE;
    }

    /* ======================= Colour Sensor Set-up ======================= */
    tcs3472 colour_sensor = TCS3472_EMPTY;
	if (tca9548a_switch_channel(&mux, MUX_CHANNEL_COLOUR_SENSOR_0) == EXIT_FAILURE) return EXIT_FAILURE;
    printf("\033[35m✔ Channel %d selected\033[0m — ready to talk to TCS3472 colour sensor on channel %d.\n", MUX_CHANNEL_COLOUR_SENSOR_0, MUX_CHANNEL_COLOUR_SENSOR_0);
    if (tcs3472Ping(&colour_sensor, INTEGRATION_TIME_MS) == EXIT_FAILURE) {
        fprintf(stderr, "\033[31m✘ Failed to initialize TCS3472 colour sensor\n\n\033[0m");
        __destroy__(&mux, EXIT_FAILURE);
        return EXIT_FAILURE;
    }




    
    /* ======================== Movement Algoritme ======================== */
    if (tca9548a_switch_channel(&mux, MUX_CHANNEL_DIST_SENSOR_0) == EXIT_FAILURE) return EXIT_FAILURE;
    
    location(r_loc_x, r_loc_y, r_angle);
    function = objectDetectionTwist(&dist_sensor, 180, &r_angle, 500);
    location(r_loc_x, r_loc_y, r_angle);
    if (function == EXIT_ERROR) {
        stepper_enable();
        printf("Stepper enables\n");
        straigth(50, 30000, &r_loc_x, &r_loc_y, &r_angle);
        printf("Stepper going straight\n");
        while(check_if_done() != 1) {
            printf("");
            if (IRborderDetection() != IR_NONE) {
                stepper_disable();
                break;
            }
        }

        if (objectDetectionTwist(&dist_sensor, 360, &r_angle, 80) == EXIT_FAILURE) return EXIT_FAILURE;
        location(r_loc_x, r_loc_y, r_angle);
    }
    else if (function == EXIT_SUCCESS) {
        if (objectScan(&dist_sensor, &mux, &colour_sensor, &r_loc_x, &r_loc_y, &r_angle)
        == EXIT_FAILURE) return EXIT_FAILURE;
        // GO AROUND
    }
    else if (function == EXIT_FAILURE) return EXIT_FAILURE;
    
    
    
    
    
    /* ====================== Destroy and exit code ======================= */
    __destroy__(&mux, EXIT_SUCCESS);

    return EXIT_SUCCESS;
}
