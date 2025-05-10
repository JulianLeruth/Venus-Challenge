#include <libpynq.h>
#include <stepper.h>
#include <stdio.h>
#include <math.h>
#include <iic.h>
#include "vl53l0x.h"
/*
  *=== Motor Size, Speed and Steps Info ===*
        Maximum speed = 3024
        This gives 30 microsec per step
            and 50 millisec per rotation
        Minimum speed = 65535
        This gives 655 microsec per step
            and 1 sec per rotation
        Wheel have diameter of ~8 cm and circonfrence of ~25 cm
        1 Rotation ~= 1600 Steps
        1 cm ~= 64 steps
*/
// ================ Naming Conventions ================ //
// Use underscores while naming variables ("use_underscores_please")
// Use lowerCamelCase while naming functions ("useThisForFunctions")
// Use all CAPS while naming fixed variables/definitions ("WIDTH" or "HEIGHT")
// Use temp_ while naming temporary variables ("temp_speed")
// Avoid using one or two letter variables ("velocity" instead of "v")

#define PI 3.14159265358979323846
#define STRAIGTH_STEPS_IN_CM 64     // Max number of steps ~= 32767
#define TURN_STEPS_IN_HALF_CM 32    // Max number of steps ~= 32767
#define MOVEMENT_SPEED 15000
#define ROTATION_SPEED 15000
#define ROBOT_WIDTH 12.3

#define NUMBER_OF_MEASUREMENT_FOR_TURNING 70

// Basis Value Functions
int sizeOfArray(int* arr) {
  int size = 0;
  while(arr[size] != '\0') size++;
  return size;
}

int highestValueIndex(int* arr) {
  int index = 0;
  for (int i = 1; i < NUMBER_OF_MEASUREMENT_FOR_TURNING; i++) {
    if (arr[0] == 0) {
      index = 1;
      i++;
    }
    if (arr[i] > arr[index] && arr[i] != 0) index = i;
  }
  return index;
}

int lowestValueIndex(int* arr) {
  int index = 0;
  for (int i = 1; i < NUMBER_OF_MEASUREMENT_FOR_TURNING; i++) {
    if (arr[0] == 0) {
      index = 1;
      i++;
    }
    if (arr[i] < arr[index] && arr[i] != 0) index = i;
  }
  return index;
}


// Movement check functions
void waitTillDone(void){
  int16_t TEMP_right = 0;
  int16_t TEMP_left = 0;
  do {
    stepper_get_steps(&TEMP_right, &TEMP_left);
    sleep_msec(50);
  } while(TEMP_right != 0 || TEMP_left != 0);
  return;
}

int checkIfDone(void){
  int16_t TEMP_right = 0;
  int16_t TEMP_left = 0;
  
  stepper_get_steps(&TEMP_right, &TEMP_left);
  if(TEMP_right != 0 || TEMP_left != 0) return 0;     // Returns 0 if not done
  else return 1;                                      // Returns 1 if done
}


// Basic Movement Funtions
void straigth(int dist /* Defined in cm */){
  stepper_enable();
  stepper_set_speed(MOVEMENT_SPEED, MOVEMENT_SPEED);

  stepper_steps(STRAIGTH_STEPS_IN_CM*dist, STRAIGTH_STEPS_IN_CM*dist);
  waitTillDone();

  stepper_disable();
  stepper_reset();
}

void turnRight(int angle /* Defined in degrees */){
  float number_of_steps = (ROBOT_WIDTH*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  stepper_enable();
  stepper_set_speed(ROTATION_SPEED, ROTATION_SPEED);
  
  printf("Float: %.1f, Int: %d", number_of_steps, (int)(number_of_steps));

  stepper_steps(0, (int)(number_of_steps));
  waitTillDone();
  
  stepper_disable();
  stepper_reset();
  sleep_msec(100);
}

void turnLeft(int angle /* Defined in degrees */){
  float number_of_steps = (ROBOT_WIDTH*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  stepper_enable();
  stepper_set_speed(ROTATION_SPEED, ROTATION_SPEED);
  
  stepper_steps((int)(number_of_steps), 0);
  waitTillDone();
  
  stepper_disable();
  stepper_reset();
  sleep_msec(100);
}

void twist(int angle /* Defined in degrees */){
  float number_of_steps = (ROBOT_WIDTH*PI*angle*STRAIGTH_STEPS_IN_CM)/360;

  stepper_enable();
  stepper_set_speed(ROTATION_SPEED, ROTATION_SPEED);

  stepper_steps(-(int)(number_of_steps), (int)(number_of_steps));
}

void straveRight(int angle, int dist /* Defined in degrees */){
  float number_of_steps_left = ((ROBOT_WIDTH + dist)*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  float number_of_steps_right = (dist*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  float rotation_speed_left = ROTATION_SPEED;
  float rotation_speed_right = (ROBOT_WIDTH + dist)/dist*rotation_speed_left;

  stepper_enable();
  stepper_set_speed(rotation_speed_left, rotation_speed_right);
  
  stepper_steps((int)(number_of_steps_left), (int)(number_of_steps_right));
  waitTillDone();
  
  stepper_disable();
  stepper_reset();
  sleep_msec(100);
}

void straveLeft(int angle, int dist /* Defined in degrees */){
  float number_of_steps_right = ((ROBOT_WIDTH + dist)*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  float number_of_steps_left = (dist*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  float rotation_speed_right = ROTATION_SPEED;
  float rotation_speed_left = (ROBOT_WIDTH + dist)/dist*rotation_speed_right;

  stepper_enable();
  stepper_set_speed(rotation_speed_left, rotation_speed_right);

  stepper_steps((int)(number_of_steps_left), (int)(number_of_steps_right));
  waitTillDone();

  stepper_disable();
  stepper_reset();
  sleep_msec(100);
}

void dance(void){
  turnRight(18);
  for(int i = 0; i < 5; i++){
    turnLeft(30);
    turnRight(30);
  }
  turnLeft(18);
  twist(-90);
  straigth(25);
  turnRight(90);
  twist(180);
  straveLeft(30,10);
  for(int i = 0; i < 5; i++){
    straveRight(-60,10);
    straveLeft(60,10);
  }
  straveRight(-30,10);
}


// Advanced Movement and detection functions
int sizeDetection(vl53x sensor) {
	int32_t iDistance;
	int32_t prevDistance;
	int number_of_measurements = 2;
	
	iDistance = tofReadDistance(&sensor);
	do {
		prevDistance = iDistance;
		iDistance = tofReadDistance(&sensor);
		printf("Left: Distance = %dmm\n", iDistance);
		stepper_steps(-16, 16);
		waitTillDone();
	} while(iDistance - prevDistance < 100);
	
	stepper_disable();
	stepper_reset();
	stepper_enable();
	
	sleep_msec(500);
	stepper_steps(32, -32);
	sleep_msec(500);
	iDistance = tofReadDistance(&sensor);
	printf("Corner Found!\nDistance = %dmm\n", iDistance);
	stepper_steps(32, -32);
	waitTillDone();
	
	do {
		prevDistance = iDistance;
		iDistance = tofReadDistance(&sensor);
		printf("Right: Distance = %dmm\n", iDistance);
		stepper_steps(16, -16);
		waitTillDone();
		number_of_measurements++;
	} while(iDistance < 500 && iDistance - prevDistance < 100);
	
	stepper_disable();
	stepper_reset();
	stepper_enable();
	
	printf("Other Corner Found!\nDistance = %dmm\nNumber of Measurements = %d\n\n", iDistance, number_of_measurements);
	
	if (number_of_measurements >= 11 && number_of_measurements <= 13) return 1;
	else if (number_of_measurements >= 14 && number_of_measurements <= 18) return 2;
	else if (number_of_measurements > 18) return 3;

 	return 0;
}

void objectDectection(vl53x sensor) {
	int32_t iDistance;
	int32_t iDistanceArr[NUMBER_OF_MEASUREMENT_FOR_TURNING+1];
	
	twist(-360);
	for(int i = 0; i < NUMBER_OF_MEASUREMENT_FOR_TURNING; i++) {
		iDistanceArr[i] =  tofReadDistance(&sensor);
		printf("1: Distance = %dmm\n", iDistanceArr[i]);
	}
	waitTillDone();
	
	int lowest_distance_index = lowestValueIndex(iDistanceArr);
	
	printf("lowest distance index: %d\nlowest distance: %d\nArr: [%d", lowest_distance_index, iDistanceArr[lowest_distance_index], iDistanceArr[0]);
	for(int i = 1; i < NUMBER_OF_MEASUREMENT_FOR_TURNING; i++) printf(", %d", iDistanceArr[i]);
	printf("]\n");
	twist(360);
	
	do {
		iDistance =  tofReadDistance(&sensor);
		printf("2: Distance = %dmm\n", iDistance);
	} while(-25 > iDistance - iDistanceArr[lowest_distance_index] || iDistance - iDistanceArr[lowest_distance_index] > 25);
	  
	stepper_disable();
	stepper_reset();
	stepper_enable();
}
