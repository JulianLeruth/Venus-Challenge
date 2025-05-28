#include <libpynq.h>
#include <stepper.h>
#include <stdio.h>
#include <math.h>
#include <iic.h>
#include "vl53l0x.h"
#include "distance_sensor.h"

/*
*=== Motor Size, Speed and Steps Info ===*
        Maximum speed = 3024
        Minimum speed = 65535
        Wheel have diameter of ~8 cm and circonfrence of ~25 cm
        1 Rotation ~= 1600 Steps
        1 cm ~= 64 steps
*/

/*
    // ================= Naming Conventions ================== //
    Use underscores while naming variables ("use_underscores_please")
    Use lowerCamelCase while naming functions ("useThisForFunctions")
    Use all CAPS while naming fixed variables/definitions ("WIDTH" or "HEIGHT")
    Use temp_ while naming temporary variables ("temp_speed")
    Avoid using one or two letter variables ("velocity" instead of "v")
                                                                        */

#define PI 3.14159265358979323846
#define ROBOT_WIDTH 12.3
#define STRAIGTH_STEPS_IN_CM 64                     // Max number of steps ~= 32767
#define NUMBER_OF_MEASUREMENT_FOR_TURNING 120
#define NUMBER_OF_DIST_MEASUREMENT_PER_SECOND 20

#define SMALL_BLOCK_MIN_NUM_MEASUREMENTS 5          // The minimum number of measurements it should take for the robot to detect a small block.
#define SMALL_BLOCK_MAX_NUM_MEASUREMENTS 13         // The maximum number of measurements it can take for the robot to detect a small block and the minimum for the robot to detect a big block.
#define BIG_BLOCK_MAX_NUM_MEASUREMENTS 25           // The maximum number of measurements it can take for the robot to detect a big block.


// ===================== Basis Value Functions ====================== //
/* The 'sizeOfArray' function measures the given array and returns the size. */
int sizeOfArray(int* arr) {
    int size = 0;
    while(arr[size] != '\0') size++;
    return size;
}

/* The 'highestValueIndex' function checks the given array and returns the index of the highest number. */  
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

/* The 'lowestValueIndex' function checks the given array and returns the index of the lowest number. */
int lowestValueIndex(int* arr) {
    int index = 0;
    for (int i = 1; i < NUMBER_OF_MEASUREMENT_FOR_TURNING; i++) {
        if (arr[0] <= 100) {
        index = 1;
        i++;
        }
        if (arr[i] < arr[index] && arr[i] > 50) index = i;
    }
    return index;
}

/* The 'convoluteArr' function takes an average of the value before, the value of and the value after an index of the array. */
void convoluteArr(int* arr) {
    for(int i = 1; i < NUMBER_OF_MEASUREMENT_FOR_TURNING-1; i++) {
        arr[i] = (arr[i-1]+arr[i]+arr[i+1])/3;
    }
}

// ==================== Movement check functions ==================== //
/* The 'waitTillDone' function stops the code until all steps are taken by the stepper motor. */
void waitTillDone(void){
    int16_t TEMP_right = 0;
    int16_t TEMP_left = 0;
    do {
        stepper_get_steps(&TEMP_right, &TEMP_left);
        sleep_msec(5);
    } while(TEMP_right != 0 || TEMP_left != 0);
    return;
}

/* The 'checkIfDone' function check and returns 1 if all steps are taken by the stepper motor and 0 if the stepper motor is not done yet. */
int check_if_done(void){
    int16_t TEMP_right = 0;
    int16_t TEMP_left = 0;

    stepper_get_steps(&TEMP_right, &TEMP_left);
    if(TEMP_right != 0 || TEMP_left != 0) return 0;     // Returns 0 if not done
    else return 1;                                      // Returns 1 if done
}

void motorReset() {
    stepper_disable();
    stepper_steps_done();
    stepper_reset();
    stepper_enable();
}

// ==================== Basic Movement Funtions ===================== //
/* The 'straight' function makes the robot go straight for a given amount of distance and a given speed. */
void straigth(int dist /* Defined in cm */, uint16_t speed){
    stepper_enable();
    stepper_set_speed(speed, speed);

    stepper_steps(STRAIGTH_STEPS_IN_CM*dist, STRAIGTH_STEPS_IN_CM*dist);
}

/* The 'turnRight' function makes the robot make a sharp turn to the right for a given amount of degrees and a given speed. */
void turnRight(int angle /* Defined in degrees */, uint16_t speed){
    float number_of_steps = (ROBOT_WIDTH*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    stepper_enable();
    stepper_set_speed(speed, speed);

    printf("Float: %.1f, Int: %d", number_of_steps, (int)(number_of_steps));

    stepper_steps(0, (int)(number_of_steps));
}

/* The 'turnLeft' function makes the robot make a sharp turn to the left for a given amount of degrees and a given speed. */
void turnLeft(int angle /* Defined in degrees */, uint16_t speed){
    float number_of_steps = (ROBOT_WIDTH*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    stepper_enable();
    stepper_set_speed(speed, speed);

    stepper_steps((int)(number_of_steps), 0);
}

/* The 'twist' function makes the robot turn around it's own axes for a given amount of degrees and a given speed. */
void twist(int angle /* Defined in degrees */, uint16_t speed){
    float number_of_steps = (ROBOT_WIDTH*PI*angle*STRAIGTH_STEPS_IN_CM)/360;

    stepper_enable();
    stepper_set_speed(speed, speed);

    stepper_steps(-(int)(number_of_steps), (int)(number_of_steps));
}

/* The 'straveRight' function makes the robot make a wide turn to the right for a given amount of degrees and a given speed. */
void straveRight(int angle, int dist /* Defined in degrees */, uint16_t speed){
    float number_of_steps_left = ((ROBOT_WIDTH + dist)*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    float number_of_steps_right = (dist*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    float rotation_speed_left = speed;
    float rotation_speed_right = (ROBOT_WIDTH + dist)/dist*rotation_speed_left;

    stepper_enable();
    stepper_set_speed(rotation_speed_left, rotation_speed_right);

    stepper_steps((int)(number_of_steps_left), (int)(number_of_steps_right));
}

/* The 'straveLeft' function makes the robot make a wide turn to the left for a given amount of degrees and a given speed. */
void straveLeft(int angle, int dist /* Defined in degrees */, uint16_t speed){
    float number_of_steps_right = ((ROBOT_WIDTH + dist)*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    float number_of_steps_left = (dist*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    float rotation_speed_right = speed;
    float rotation_speed_left = (ROBOT_WIDTH + dist)/dist*rotation_speed_right;

    stepper_enable();
    stepper_set_speed(rotation_speed_left, rotation_speed_right);

    stepper_steps((int)(number_of_steps_left), (int)(number_of_steps_right));
}

/* The 'dance' function makes the robot do a little dance ;) */
void dance(void){
    uint16_t speed = 15000;
    turnRight(18, speed);
    for(int i = 0; i < 5; i++){
        turnLeft(30, speed);
        turnRight(30, speed);
    }
    turnLeft(18, speed);
    twist(-90, speed);
    straigth(25, speed);
    turnRight(90, speed);
    twist(180, speed);
    straveLeft(30,10, speed);
    for(int i = 0; i < 5; i++){
        straveRight(-60,10, speed);
        straveLeft(60,10, speed);
    }
    straveRight(-30,10, speed);
}


// =========== Advanced Movement and detection functions ============ //
/* The 'sizeDetection' function measures the size of an object and returns either 1, 2, 3 based on which size the object is */
int sizeDetection(vl53x sensor) {
    int32_t iDistance;
    int32_t prevDistance;
    int number_of_measurements = 2;
    int number_of_dist_measurements = 0;
    iDistance = tofReadDistance(&sensor);
    do {
        prevDistance = iDistance;
        iDistance = tofReadDistance(&sensor);
        printf("Left: Distance = %dmm\n", iDistance);
        stepper_steps(-16, 16);
        waitTillDone();
        number_of_dist_measurements++;
        if (number_of_dist_measurements % 50 == 0) {
            printf("Flushed\n");
            iic_destroy(IIC0);
            iic_init(IIC0);
            if (vl53l0xFlush(&sensor) == -1) return -1;
        }
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
    
    motorReset();

    printf("Other Corner Found!\nDistance = %dmm\nNumber of Measurements = %d\n\n", iDistance, number_of_measurements);

    if (number_of_measurements >= SMALL_BLOCK_MIN_NUM_MEASUREMENTS
        && number_of_measurements <= SMALL_BLOCK_MAX_NUM_MEASUREMENTS)  return 1;
    else if (number_of_measurements > SMALL_BLOCK_MIN_NUM_MEASUREMENTS
        && number_of_measurements <= BIG_BLOCK_MAX_NUM_MEASUREMENTS)    return 2;
    else if (number_of_measurements > BIG_BLOCK_MAX_NUM_MEASUREMENTS)   return 3;
    return 0;
}

/* The 'objectDectection' function make a 360 degree turn and detects the closest objects to the robot and then turns back to the closest object */
int objectDetectionTwist(vl53x sensor) {
    stepper_reset();
    stepper_enable();

    int32_t iDistanceArr[NUMBER_OF_MEASUREMENT_FOR_TURNING+1];
    int loop = 0;
    int number_of_dist_measurements = 0;

    twist(-360, 63000);
    for(int i = 0; i < NUMBER_OF_MEASUREMENT_FOR_TURNING; i++) {
        iDistanceArr[i] =  tofReadDistance(&sensor);
        printf("%d: Distance = %dmm\n", i, iDistanceArr[i]);
        sleep_msec(50);
        number_of_dist_measurements++;
        if (number_of_dist_measurements % 50 == 0) {
            printf("Flushed\n");
            iic_destroy(IIC0);
            iic_init(IIC0);
            if (vl53l0xFlush(&sensor) == -1) return -1;
        }
    }
    waitTillDone();

    int lowest_distance_index = lowestValueIndex(iDistanceArr);
    printf("\n\033[0;31mlowest distance index: \033[0;37m%d\n\033[0;31mlowest distance: \033[0;37m%d mm \n\nArr: [%d", lowest_distance_index, iDistanceArr[lowest_distance_index], iDistanceArr[0]);
    for(int i = 1; i < NUMBER_OF_MEASUREMENT_FOR_TURNING; i++) printf(", %d", iDistanceArr[i]);
    printf("]\n");

    convoluteArr(iDistanceArr);
    printf("\nArr: [%d", iDistanceArr[0]);
    for(int i = 1; i < NUMBER_OF_MEASUREMENT_FOR_TURNING; i++) printf(", %d", iDistanceArr[i]);
    printf("]\n");

    // int32_t iDistance;
    // twist(360, 63000);

    // do {
    //     sleep_msec(50);
    //     loop++;
    //     iDistance =  tofReadDistance(&sensor);
    //     printf("%d: Distance = %dmm\n", NUMBER_OF_MEASUREMENT_FOR_TURNING - loop, iDistance);
    //     number_of_dist_measurements++;
    //     if (number_of_dist_measurements % 50 == 0) {
    //         printf("Flushed\n");
    //         iic_destroy(IIC0);
    //         iic_init(IIC0);
    //         if (vl53l0xFlush(&sensor) == -1) return -1;
    //     }
    // } while((-35 > iDistance - iDistanceArr[lowest_distance_index] || iDistance - iDistanceArr[lowest_distance_index] > 35) && loop < NUMBER_OF_MEASUREMENT_FOR_TURNING);
    
    // stepper_disable();
    // stepper_reset();
    // stepper_enable();
    // stepper_steps(64, -64);
    
    printf("\033[0;32m\nTurning %d degrees!\n\n\033[0;37m", 360*-lowest_distance_index/NUMBER_OF_MEASUREMENT_FOR_TURNING);
    twist(-360*lowest_distance_index/NUMBER_OF_MEASUREMENT_FOR_TURNING, 48000);
    
    if (loop == NUMBER_OF_MEASUREMENT_FOR_TURNING) return 0;
    else return 1;
}
