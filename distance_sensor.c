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
	printf("Revision ID - %d\n\n", revision);
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
