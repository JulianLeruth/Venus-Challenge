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


#define MUX_CHANNEL_DIST_SENSOR_0       0
#define MUX_CHANNEL_DIST_SENSOR_1       1
#define MUX_CHANNEL_COLOUR_SENSOR_0     7
#define MUX_CHANNEL_COLOUR_SENSOR_1     3

#define INTEGRATION_TIME_MS            60

#define BLACK                           0
#define WHITE                           1
#define RED                             2
#define BLUE                            3
#define GREEN                           4

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
    
    // float r_loc_x = 0;          // The X location of the robot, relative to the starting position
    // float r_loc_y = 0;          // The Y location of the robot, relative to the starting position
    // float robot_angle = 0;      // The angle of the robot, relative to the starting position

    /* ========================= Switchbox Set-up ========================= */
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    switchbox_set_pin(IO_AR4, SWB_GPIO);
    gpio_set_direction(IO_AR4, GPIO_DIR_OUTPUT);
    gpio_set_level(IO_AR4, GPIO_LEVEL_LOW);
    switchbox_set_pin(IO_AR5, SWB_GPIO);
    switchbox_set_pin(IO_AR6, SWB_GPIO);
    
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

    
    /* ======================== Movement Functions ======================== */
    tca9548a_switch_channel(&mux, MUX_CHANNEL_DIST_SENSOR_0);
    
	if (objectDetectionTwistPart(&dist_sensor, 180) == EXIT_FAILURE) return EXIT_FAILURE;
    if (objectAproach(&dist_sensor, 200) == EXIT_FAILURE) return EXIT_FAILURE;
    int block_size = sizeDetection(&dist_sensor);
    if (block_size == 1) printf("Block is of size 3x3x3!\n");
    else if (block_size == 2) printf("Block is of size 6x6x6!\n");
    else if (block_size == 3) printf("Mountain detected!\n");
    else if (block_size == 0) printf("Error no size detected\n");
    if (objectDetectionTwistPart(&dist_sensor, 40) == EXIT_FAILURE) return EXIT_FAILURE;
    
    if (objectAproach(&dist_sensor, 35) == EXIT_FAILURE) return EXIT_FAILURE;

    tca9548a_switch_channel(&mux, MUX_CHANNEL_COLOUR_SENSOR_0);
    
    int block_colour = colourSensor(&colour_sensor, INTEGRATION_TIME_MS);
    if (block_colour == BLACK) printf("Block is black!\n");
    else if (block_colour == WHITE) printf("Block is white!\n");
    else if (block_colour == RED) printf("Block is red!\n");
    else if (block_colour == BLUE) printf("Block is blue!\n");
    else if (block_colour == GREEN) printf("Block is green!\n");


     
    /* ====================== Destroy and exit code ======================= */
    __destroy__(&mux, EXIT_SUCCESS);

    return EXIT_SUCCESS;
}
