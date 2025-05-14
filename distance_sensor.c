#include <libpynq.h>
#include <stepper.h>
#include <stdio.h>
#include <math.h>
#include <iic.h>
#include "vl53l0x.h"
#include "movement_lib.h"

int vl53l0xExample(void) {
    int sensor_ping;
    uint8_t addr = 0x29;
	sensor_ping = tofPing(IIC0, addr);
	printf("Sensor Ping: ");
	if(sensor_ping != 0) {
		printf("Fail\n");
		return 1;
	}
	printf("Succes\n");
	// Create a sensor struct
	vl53x sensor;

	// Initialize the sensor
	sensor_ping = tofInit(&sensor, IIC0, addr, 0); // set default range mode (up to 800mm)
	if (sensor_ping != 0) {
		return -1; // problem - quit
	}
	uint8_t model, revision;

	printf("VL53L0X device successfully opened.\n");
	tofGetModel(&sensor, &model, &revision);
	printf("Model ID - %d\n", model);
	printf("Revision ID - %d\n", revision);
	fflush(NULL); //Get some output even is distance readings hang
	
	int32_t iDistance;
	int32_t prevDistance;
    
	stepper_init();
	stepper_reset();	
	stepper_enable();

	objectDetection(sensor);

	iDistance = tofReadDistance(&sensor);
	do {
		prevDistance = iDistance;
		iDistance = tofReadDistance(&sensor);
		printf("Distance = %dmm\n", iDistance);
		stepper_steps(64, 64);
	} while(iDistance > 200 || prevDistance > 200);

	int size = sizeDetection(sensor);

	if (size == 1) printf("Block is of size 3x3x3!\n");
	else if (size == 2) printf("Block is of size 6x6x6!\n");
	else if (size == 3) printf("Mountain detected!\n");
	else if (size == 0) printf("Error no size detected\n");

	stepper_disable();
	stepper_destroy();
	return EXIT_SUCCESS;
}
