#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>
#include <stepper.h>
#include <math.h>
#include "movement_lib.h"

int vl53l0xSensorCheck(void) {
	int i;
	uint8_t addr = 0x29;
	i = tofPing(IIC0, addr);
	printf("Sensor Ping: ");
	if(i != 0) {
		printf("Fail\n");
		return 1;
	}
	printf("Succes\n");
	return 0;
};

int vl53l0xExampleSingle(void) {
	if (vl53l0xSensorCheck()) return 1;
	// Create a sensor struct
	vl53x sensor;

	// Initialize the sensor
	i = tofInit(&sensor, IIC0, addr, 0); // set default range mode (up to 800mm)
	if (i != 0) {
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
	stepper_set_speed(15000*2, 15000*2);
	stepper_enable();
	
	objectDetection(sensor);

	iDistance = tofReadDistance(&sensor);
	do { // read values 20 times a second
		prevDistance = iDistance;
		iDistance = tofReadDistance(&sensor);
		printf("Distance = %dmm\n", iDistance);
		stepper_steps(64, 64);
		// sleep_msec(100);
	} while(iDistance > 200 || prevDistance > 200);

	int size = sizeDetection(sensor);

	if (size == 1) printf("Block is of size 3x3x3!\n");
	else if (size == 2) printf("Block is of size 6x6x6!\n");
	else if (size == 3) printf("Mountain detected!\n");
	else if (size == 0) printf("Error no size detected\n");

	waitTillDone();
	stepper_disable();
	stepper_destroy();
	
	return EXIT_SUCCESS;
}
