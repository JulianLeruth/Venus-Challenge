#include <libpynq.h>
#include <stepper.h>
#include <stdio.h>
#include <math.h>
#include <iic.h>
#include "vl53l0x.h"
#include "movement_lib.h"

int vl53l0xPing(vl53x* sensor) {
    int sensor_ping;
    uint8_t addr = 0x29;
    do {
        sensor_ping = tofPing(IIC0, addr);
        printf("Sensor Ping: ");
        if(sensor_ping != 0) {
		    printf("\033[31m✘ Fail\033[0m\n");
            iic_destroy(IIC0);
            iic_init(IIC0);
        } else printf("\033[32m✔ Succes\033[0m\n");
    } while(sensor_ping != 0);

	// Initialize the sensor
	sensor_ping = tofInit(sensor, IIC0, addr, 0); // set default range mode (up to 800mm)
	if (sensor_ping != 0) return EXIT_FAILURE;
	uint8_t model, revision;

	printf("VL53L0X device successfully opened.\n");
	tofGetModel(sensor, &model, &revision);
	printf("Model ID - %d\n", model);
	printf("Revision ID - %d\n", revision);
	fflush(NULL); //Get some output even is distance readings hang
    return EXIT_SUCCESS;
}

int vl53l0xFlush(vl53x* sensor) {
    int sensor_ping;
    uint8_t addr = 0x29;
    do {
        sensor_ping = tofPing(IIC0, addr);
        if(sensor_ping != 0) {
            iic_destroy(IIC0);
            iic_init(IIC0);
        }
    } while(sensor_ping != 0);

	// Initialize the sensor
	sensor_ping = tofInit(sensor, IIC0, addr, 0); // set default range mode (up to 800mm)
	if (sensor_ping != 0) return EXIT_FAILURE; // problem - quit
	fflush(NULL); //Get some output even is distance readings hang
    
    return EXIT_SUCCESS;
}

int flushIICChannel(vl53x* sensor, int measurements) {
    if (measurements >= 50) {
        printf("Flushed\n");
        iic_destroy(IIC0);
        iic_init(IIC0);
        if (vl53l0xFlush(sensor) == EXIT_FAILURE) return EXIT_FAILURE;
        return EXIT_SUCCESS;
    }
    return 0;
}

int vl53l0xTestWithGiven(vl53x* sensor) {
    int number_of_dist_measurements = 0;
	int32_t iDistance;
    for(int i = 0; i < 100; i++) {
        iDistance = tofReadDistance(sensor);
        printf("Distance: %dmm\n", iDistance);
        sleep_msec(50);
        number_of_dist_measurements++;
        int temp = flushIICChannel(sensor, number_of_dist_measurements);
        if (temp == EXIT_FAILURE) return EXIT_FAILURE;
        if (temp == EXIT_SUCCESS) number_of_dist_measurements = 0;
    }
	return EXIT_SUCCESS;
}

int vl53l0xTest(void) {
    // Create a sensor struct
	vl53x sensor;
    if(vl53l0xPing(&sensor) == EXIT_FAILURE) return EXIT_FAILURE;
    int number_of_dist_measurements = 0;
	int32_t iDistance;
    for(int i = 0; i < 100; i++) {
        iDistance = tofReadDistance(&sensor);
        printf("Distance: %dmm\n", iDistance);
        sleep_msec(50);
        number_of_dist_measurements++;
        int temp = flushIICChannel(&sensor, number_of_dist_measurements);
        if (temp == EXIT_FAILURE) return EXIT_FAILURE;
        if (temp == EXIT_SUCCESS) number_of_dist_measurements = 0;
    }
	return EXIT_SUCCESS;
}

int vl53l0xExample(void) {
    // Create a sensor struct
	vl53x sensor;
    if(vl53l0xPing(&sensor) == EXIT_FAILURE) return EXIT_FAILURE;

    int number_of_dist_measurements = 0;
	int32_t iDistance;
	int32_t prev_distance;
    
	stepper_init();
	stepper_reset();	
	stepper_enable();
	if(objectDetectionTwist360(&sensor)) {
        stepper_set_speed(45000, 45000);
        iDistance = tofReadDistance(&sensor);
        do {
            prev_distance = iDistance;
            iDistance = tofReadDistance(&sensor);
            printf("Distance = %dmm\n", iDistance);
            stepper_steps(64, 64);
            sleep_msec(50);
            number_of_dist_measurements++;
            int temp = flushIICChannel(&sensor, number_of_dist_measurements);
            if (temp == EXIT_FAILURE) return EXIT_FAILURE;
            if (temp == EXIT_SUCCESS) number_of_dist_measurements = 0;
        } while(iDistance > 200 || prev_distance > 200);

        int size = sizeDetection(&sensor);

        if (size == 1) printf("Block is of size 3x3x3!\n");
        else if (size == 2) printf("Block is of size 6x6x6!\n");
        else if (size == 3) printf("Mountain detected!\n");
        else if (size == 0) printf("Error no size detected\n");
    }
	stepper_disable();
	stepper_destroy();
	return EXIT_SUCCESS;
}
