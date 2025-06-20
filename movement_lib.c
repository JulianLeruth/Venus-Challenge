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

// ===================== Basis Value Functions ====================== //
int sizeOfArray(int* arr) {
    int size = 0;
    while(arr[size] != '\0') size++;
    return size;
}

int highestValueIndex(int* arr) {
    int index = 0;
    for (int i = 1; i < sizeOfArray(arr); i++) {
        if (arr[0] == 0) {
        index = 1;
        i++;
        }
        if (arr[i] > arr[index] && arr[i] != 0) index = i;
    }
    return index;
}

int lowestValueIndex(int* arr) {
    int index = -1;
    for (int i = 0; i < sizeOfArray(arr); i++) {
        if (arr[i] < arr[index] && arr[i] > 50 && arr[i] < 3000) index = i;
    }
    return index;
}

void convoluteArr(int* arr) {
    int temp_arr[sizeOfArray(arr)];
    for(int i = 0; i < sizeOfArray(arr); i++) temp_arr[i] = arr[i];

    if (temp_arr[0] <= 30) arr[0] = 8192;
    if (temp_arr[1] <= 30) arr[1] = 8192;
    for(int i = 1; i < sizeOfArray(arr)-1; i++) {
        if (temp_arr[i+1] <= 20) arr[i+1] = 8192;
        arr[i] = (temp_arr[i-1]+temp_arr[i]+temp_arr[i+1])/3;
    }
}

double cosin(double angle) {
    return cos(angle*PI/180);
}

double sinus(double angle) {
    return sin(angle*PI/180);
}



// ===================== Localization Functions ===================== //
void blockLocation(double r_loc_x, double r_loc_y, double r_angle, int32_t dist, double* b_loc_x, double* b_loc_y) {
    *b_loc_x = r_loc_x + (dist/10)*cosin(r_angle);
    *b_loc_y = r_loc_y + (dist/10)*sinus(r_angle);
}

void locationPrint(double r_loc_x, double r_loc_y, double r_angle) {
    printf("\033[35mLocation: \033[0m(%.1f, %.1f),\033[32m Angle:\033[0m %.1f\n", r_loc_x, r_loc_y, r_angle);
}

void locationRecalculation(double* r_loc_x, double* r_loc_y, double* r_angle) {
    stepper_disable();

    int16_t left = 0, right = 0;
    stepper_get_steps(&left, &right);
    // printf("Stepper: %d steps, %d steps\n", left, right);

    if (left != 0 || right != 0) {
        double left_dist = left/STRAIGTH_STEPS_IN_CM;
        // double right_dist = right/STRAIGTH_STEPS_IN_CM;
        
        // printf("Stepper: %.1fcm, %.1fcm\n", left_dist, right_dist);
        if (left == right) {                                                        // Last movement was straight
            *r_loc_x -= left_dist * cosin(*r_angle);
            *r_loc_y -= left_dist * sinus(*r_angle);
        }
        else if (left == -right) {                                                  // Last movement was a twist
            *r_angle += left*360/(ROBOT_WIDTH*PI*STRAIGTH_STEPS_IN_CM);
            if (*r_angle >= 360) *r_angle -= 360;
            else if (*r_angle < 0) *r_angle += 360;
        }
        else if (left != 0 && right == 0) {                                         // Last movement was a right turn
            double angle = (left*360)/(ROBOT_WIDTH*2*PI*STRAIGTH_STEPS_IN_CM);
            double radius = ROBOT_WIDTH / 2.0;
            double dx = radius * (1 - cosin(angle));
            double dy = radius * sinus(angle);

            *r_loc_x += dx;
            *r_loc_y -= dy;
            
            *r_angle -= angle;
            if (*r_angle >= 360) *r_angle -= 360;
            else if (*r_angle < 0) *r_angle += 360;
        }
        else if (left == 0 && right != 0) {                                         // Last movement was a left turn
            double angle = (right*360)/(ROBOT_WIDTH*2*PI*STRAIGTH_STEPS_IN_CM);
            double radius = ROBOT_WIDTH / 2.0;
            double dx = radius * (1 - cosin(angle));
            double dy = radius * sinus(angle);

            *r_loc_x -= dx;
            *r_loc_y -= dy;

            *r_angle -= angle;
            if (*r_angle >= 360) *r_angle -= 360;
            else if (*r_angle < 0) *r_angle += 360;
        }
    }
    stepper_reset();
    stepper_enable();
    locationPrint(*r_loc_x, *r_loc_y, *r_angle);
}



// ==================== Movement check functions ==================== //
void waitTillDone(void) {
    int16_t TEMP_right = 0;
    int16_t TEMP_left = 0;
    do {
        stepper_get_steps(&TEMP_right, &TEMP_left);
        sleep_msec(5);
    } while(abs(TEMP_right) > 0 || abs(TEMP_left) > 0);
    return;
}

int checkIfDone(void) {
    int16_t TEMP_right = 0;
    int16_t TEMP_left = 0;

    stepper_get_steps(&TEMP_right, &TEMP_left);
    if(TEMP_right != 0 || TEMP_left != 0) return 1;     // Returns 1 if not done
    else return 0;                                      // Returns 0 if done
}



// ==================== Basic Movement Funtions ===================== //
void straigth(double dist, uint16_t speed, double* r_loc_x, double* r_loc_y, double* r_angle) {
    stepper_enable();
    stepper_set_speed(speed, speed);

    stepper_steps(STRAIGTH_STEPS_IN_CM*dist, STRAIGTH_STEPS_IN_CM*dist);

    *r_loc_x += dist * cosin(*r_angle);
    *r_loc_y += dist * sinus(*r_angle);
}

void twist(double angle, uint16_t speed, double* r_angle) {
    double number_of_steps = (ROBOT_WIDTH*PI*angle*STRAIGTH_STEPS_IN_CM)/360;

    stepper_enable();
    stepper_set_speed(speed, speed);

    stepper_steps(-(int)(number_of_steps), (int)(number_of_steps));

    *r_angle += angle;
    if (*r_angle >= 360) *r_angle -= 360;
    else if (*r_angle < 0) *r_angle += 360;
}

void turnRight(double angle, uint16_t speed, double* r_loc_x, double* r_loc_y, double* r_angle) {
    double number_of_steps = (ROBOT_WIDTH*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    stepper_enable();
    stepper_set_speed(speed, speed);

    stepper_steps(0, (int)(number_of_steps));

    *r_angle += angle;
    if (*r_angle >= 360) *r_angle -= 360;
    else if (*r_angle < 0) *r_angle += 360;

    double radius = ROBOT_WIDTH / 2.0;
    double dx = radius * (1 - cosin(angle));
    double dy = radius * sinus(angle);

    *r_loc_x += dx;
    *r_loc_y += dy;
}

void turnLeft(double angle, uint16_t speed, double* r_loc_x, double* r_loc_y, double* r_angle) {
    double number_of_steps = (ROBOT_WIDTH*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    stepper_enable();
    stepper_set_speed(speed, speed);

    stepper_steps((int)(number_of_steps), 0);

    *r_angle += angle;
    if (*r_angle >= 360) *r_angle -= 360;
    else if (*r_angle < 0) *r_angle += 360;

    double radius = ROBOT_WIDTH / 2.0;
    double dx = radius * (1 - cosin(angle));
    double dy = radius * sinus(angle);

    *r_loc_x -= dx;
    *r_loc_y += dy;
}

void straveRight(double angle, double dist, uint16_t speed, double* r_loc_x, double* r_loc_y, double* r_angle) {
    double number_of_steps_left = ((ROBOT_WIDTH + dist)*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    double number_of_steps_right = (dist*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    double rotation_speed_right = speed;
    double rotation_speed_left = dist/(ROBOT_WIDTH + dist)*speed;

    stepper_enable();
    stepper_set_speed((int)(rotation_speed_left), (int)(rotation_speed_right));

    stepper_steps((int)(number_of_steps_left), (int)(number_of_steps_right));

    *r_angle += angle;
    if (*r_angle >= 360) *r_angle -= 360;
    else if (*r_angle < 0) *r_angle += 360;

    double radius = dist + (ROBOT_WIDTH / 2.0);
    double dx = radius * (1 - cosin(angle));
    double dy = radius * sinus(angle);

    *r_loc_x += dx;
    *r_loc_y += dy;
}

void straveLeft(double angle, double dist, uint16_t speed, double* r_loc_x, double* r_loc_y, double* r_angle) {
    double number_of_steps_right = ((ROBOT_WIDTH + dist)*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    double number_of_steps_left = (dist*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
    double rotation_speed_left = speed;
    double rotation_speed_right = dist/(ROBOT_WIDTH + dist)*speed;

    stepper_enable();
    stepper_set_speed(rotation_speed_left, rotation_speed_right);

    stepper_steps((int)(number_of_steps_left), (int)(number_of_steps_right));

    *r_angle += angle;
    if (*r_angle >= 360) *r_angle -= 360;
    else if (*r_angle < 0) *r_angle += 360;

    double radius = dist + (ROBOT_WIDTH / 2.0);
    double dx = radius * (1 - cosin(angle));
    double dy = radius * sinus(angle);

    *r_loc_x -= dx;
    *r_loc_y += dy;
}

void avoidBorder(double* r_loc_x, double* r_loc_y, double* r_angle) {
    straigth(-5, 30000, r_loc_x, r_loc_y, r_angle);
    waitTillDone();
    locationRecalculation(r_loc_x, r_loc_y, r_angle);
    twist(90, 30000, r_angle);
    waitTillDone();
    locationRecalculation(r_loc_x, r_loc_y, r_angle);
    straigth(ROBOT_WIDTH, 30000, r_loc_x, r_loc_y, r_angle);
    waitTillDone();
    locationRecalculation(r_loc_x, r_loc_y, r_angle);
    twist(90, 30000, r_angle);
    waitTillDone();
    locationRecalculation(r_loc_x, r_loc_y, r_angle);
}

void avoidCliff(double* r_loc_x, double* r_loc_y, double* r_angle) {
    straigth(-10, SPEED_STRAIGHT_MED, r_loc_x, r_loc_y, r_angle);
    waitTillDone();
    twist(90, SPEED_TWIST_FAST, r_angle);
    waitTillDone();
    straveRight(180, 15, SPEED_TURN_SLOW, r_loc_x, r_loc_y, r_angle);
    waitTillDone();
    twist(90, SPEED_TWIST_FAST, r_angle);
    waitTillDone();
}

void avoidBlock(double* r_loc_x, double* r_loc_y, double* r_angle) {
    straigth(-8, SPEED_STRAIGHT_MED, r_loc_x, r_loc_y, r_angle);
    waitTillDone();
    twist(90, SPEED_TWIST_FAST, r_angle);
    waitTillDone();
    straveRight(180, 10, SPEED_TURN_SLOW, r_loc_x, r_loc_y, r_angle);
    waitTillDone();
    twist(90, SPEED_TWIST_FAST, r_angle);
    waitTillDone();
}



// ======================= Detection functions ====================== //
int sizeDetection(vl53x* sensor, double* r_angle) {
    int32_t iDistance;
    int32_t prevDistance;

    int number_of_measurements = 2;
    int number_of_dist_measurements = 0;

    stepper_set_speed(SPEED_TWIST_MED, SPEED_TWIST_MED);
    printf("\n\n");

    iDistance = tofReadDistance(sensor);
    do {
        prevDistance = iDistance;
        iDistance = tofReadDistance(sensor);
        printf("\033[1FDistance = %dmm\n", iDistance);
        stepper_steps(-16, 16);
        *r_angle += (double)(16*360)/(ROBOT_WIDTH*PI*STRAIGTH_STEPS_IN_CM);
        waitTillDone();
        number_of_dist_measurements++;
        int temp = flushIICChannel(sensor, number_of_dist_measurements);
        if (temp == EXIT_FAILURE) return EXIT_FAILURE;
        if (temp == EXIT_SUCCESS) number_of_dist_measurements = 0;
    } while(iDistance - prevDistance < 100 || (iDistance <= 400 && prevDistance <= 400));

    stepper_disable();
    stepper_reset();
    stepper_enable();

    sleep_msec(500);
    stepper_steps(32, -32);
    *r_angle += (double)(-32*360)/(ROBOT_WIDTH*PI*STRAIGTH_STEPS_IN_CM);
    sleep_msec(500);
    iDistance = tofReadDistance(sensor);
    printf("Corner Found!\n\n\n");
    stepper_steps(32, -32);
    *r_angle += (double)(-32*360)/(ROBOT_WIDTH*PI*STRAIGTH_STEPS_IN_CM);
    waitTillDone();
    
    do {
        prevDistance = iDistance;
        iDistance = tofReadDistance(sensor);
        printf("\033[1FDistance = %dmm\n", iDistance);
        stepper_steps(16, -16);
        *r_angle += (double)(-16*360)/(ROBOT_WIDTH*PI*STRAIGTH_STEPS_IN_CM);
        waitTillDone();
        number_of_measurements++;
        number_of_dist_measurements++;
        int temp = flushIICChannel(sensor, number_of_dist_measurements);
        if (temp == EXIT_FAILURE) return EXIT_FAILURE;
        if (temp == EXIT_SUCCESS) number_of_dist_measurements = 0;
    } while(iDistance < 400 && iDistance - prevDistance < 100);
    
    stepper_disable();
    stepper_reset();
    stepper_enable();  
    
    printf("Other Corner Found!\nNumber of Measurements = %d\n\n", number_of_measurements);
    
    /* === Safety Flush === */
    printf("Flushed\n");
    iic_destroy(IIC0);
    iic_init(IIC0);
    if (vl53l0xFlush(sensor) == -1) return EXIT_FAILURE;

    if (number_of_measurements >= SMALL_BLOCK_MIN_NUM_MEASUREMENTS
        && number_of_measurements <= SMALL_BLOCK_MAX_NUM_MEASUREMENTS)  return SMALL_BLOCK;
    else if (number_of_measurements > SMALL_BLOCK_MIN_NUM_MEASUREMENTS
        && number_of_measurements <= BIG_BLOCK_MAX_NUM_MEASUREMENTS)    return BIG_BLOCK;
    else if (number_of_measurements > BIG_BLOCK_MAX_NUM_MEASUREMENTS)   return MOUNTAIN;
    else                                                                return NO_BLOCK;
} /* int sizeDetection(vl53x* sensor, double* r_angle); */

int objectDetectionTwist(vl53x* sensor, int angle, double* r_angle, int max_dist) {
    /* === Initiation === */
    stepper_reset();
    stepper_enable();
    int number_of_measurements = (float)(NUMBER_OF_MEASUREMENT_FOR_TURNING_360*angle)/360.0;

    int32_t iDistanceArr[angle+1];
    int number_of_dist_measurements = 0;
    if (angle != 360) {
        twist(angle/2, SPEED_TWIST_FAST, r_angle);
        waitTillDone();
    }
    
    for(int i = 0; i < 5; i++) tofReadDistance(sensor);

    /* === First measurement turning circle === */
    twist(-angle, SPEED_TWIST_SLOW, r_angle);
    printf("\033[34mScanning Area for objects.\033[0m\n\n\n");


    for(int i = 0; i < number_of_measurements; i++) {
        iDistanceArr[i] =  tofReadDistance(sensor);
        if (iDistanceArr[i] > max_dist) iDistanceArr[i] = 8192;
        printf("\033[1F%d: Distance = %dmm\n", i, iDistanceArr[i]);
        sleep_msec(50);
        number_of_dist_measurements++;
        int temp = flushIICChannel(sensor, number_of_dist_measurements);
        if (temp == EXIT_FAILURE) return EXIT_FAILURE;
        if (temp == EXIT_SUCCESS) number_of_dist_measurements = 0;
    }
    waitTillDone();


    // printf("\nArr: [%d", iDistanceArr[0]);
    // for(int i = 1; i < number_of_measurements; i++) printf(", %d", iDistanceArr[i]);
    // printf("]\n");

    /* === Convolution of the measurements === */
    convoluteArr(iDistanceArr);
    int lowest_distance_index = lowestValueIndex(iDistanceArr);
    // printf("\n\033[31mlowest distance index: \033[0m%d\n\033[31mlowest distance: \033[0m%d mm \n\nArr: [%d", lowest_distance_index, iDistanceArr[lowest_distance_index], iDistanceArr[0]);
    // for(int i = 1; i < number_of_measurements; i++) {
    //     if (i == lowest_distance_index) printf(", \033[35m%d\033[0m", iDistanceArr[i]);
    //     else printf(", %d", iDistanceArr[i]);
    // }
    // printf("]\n");
    
    /* === Turn back to the lowest value === */
    if (lowest_distance_index == -1) {
        int turningAngle = angle/2;
        printf("\033[31mNo block found.\033[0m\nTurning %d degrees!\033[0m\n", turningAngle);
        
        if (angle != 360) {
            twist(turningAngle, SPEED_TWIST_FAST, r_angle);
            waitTillDone();
        }

        /* === Safety Flush === */
        printf("Flushed\n");
        iic_destroy(IIC0);
        iic_init(IIC0);

        if (vl53l0xFlush(sensor) == EXIT_FAILURE) return EXIT_FAILURE;
        else return EXIT_ERROR;
    }
    else {
        int turningAngle = angle*(number_of_measurements - lowest_distance_index)/number_of_measurements;
        printf("\033[32m\nTurning %d degrees!\n\n\033[0m", turningAngle);
        twist(turningAngle, SPEED_TWIST_MED, r_angle);
        waitTillDone();

        /* === Safety Flush === */
        printf("Flushed\n");
        iic_destroy(IIC0);
        iic_init(IIC0);

        if (vl53l0xFlush(sensor) == EXIT_FAILURE) return EXIT_FAILURE;
        else return EXIT_SUCCESS;
    }
} /* int objectDetectionTwist(vl53x* sensor, int angle, double* r_angle); */

int objectAproach(vl53x* sensor, int max_dist, double* r_loc_x, double* r_loc_y, double* r_angle) {
    stepper_reset();
    stepper_enable();
    
    int number_of_dist_measurements = 0;
	int32_t iDistance;
	int32_t prev_distance;
    
    stepper_set_speed(SPEED_STRAIGHT_SLOW, SPEED_STRAIGHT_SLOW);
    printf("\033[34mAproaching object until %dmm.\033[0m\n", max_dist);
    iDistance = tofReadDistance(sensor);
    do {
        prev_distance = iDistance;
        iDistance = tofReadDistance(sensor);
        //printf("Distance = %dmm\n", iDistance);
        stepper_steps(64, 64);
        *r_loc_x += cos(*r_angle);
        *r_loc_y += sin(*r_angle);
        sleep_msec(50);
        number_of_dist_measurements++;
        int temp = flushIICChannel(sensor, number_of_dist_measurements);
        if (temp == EXIT_FAILURE) return EXIT_FAILURE;
        if (temp == EXIT_SUCCESS) number_of_dist_measurements = 0;
        if (IRborderDetection() != IR_NONE) {
            printf("\033[31mBorder Detected. Stepper Disabled!\033[0m\n");
            stepper_disable();
            return EXIT_SUCCESS;
        }
    } while(iDistance > max_dist || prev_distance > max_dist);
    return EXIT_SUCCESS;
} /* int objectAproach(vl53x* sensor, int max_dist, double* r_loc_x, double* r_loc_y, double* r_angle); */

int objectScan(vl53x* dist_sensor, tca9548a* mux, tcs3472* colour_sensor, double* r_loc_x, double* r_loc_y, double* r_angle) {
    double b_loc_x = 0;             // The X location of an object, relative to the robots position
    double b_loc_y = 0;             // The Y location of an object, relative to the robots position
    int32_t averageDist = 0;

    for(int i = 0; i < 20; i++) {
        averageDist += tofReadDistance(dist_sensor);
        sleep_msec(50);
    }
    averageDist /= 20;

    printf("\033[32mLocation of robot:\033[0m (%.1f; %.1f), %.1f\n\033[32mDistance to object:\033[0m %dmm\n", *r_loc_x, *r_loc_y, *r_angle, averageDist);
    blockLocation(*r_loc_x, *r_loc_x, *r_angle, averageDist, &b_loc_x, &b_loc_y);
    printf("\033[32mEstimate object location: \033[0m(%.1f; %.1f)\n", b_loc_x, b_loc_y);
    

    //
    // Asking the server if the block is already registered
    //

    
    if (objectAproach(dist_sensor, 200, r_loc_x, r_loc_y, r_angle) == EXIT_FAILURE) return EXIT_FAILURE;
    locationPrint(*r_loc_x, *r_loc_y, *r_angle);
    
    
    int block_size;
    block_size = sizeDetection(dist_sensor, r_angle);
    printf("\033[35mLocation: \033[0m(%.1f, %.1f),\033[32m Angle:\033[0m %.1f\n", *r_loc_x, *r_loc_y, *r_angle);
        
    twist(20, SPEED_TWIST_FAST, r_angle);
    waitTillDone();
    
    if (block_size == EXIT_FAILURE) return EXIT_FAILURE;
    else if (block_size == SMALL_BLOCK) printf("Block is of size 3x3x3!\n");
    else if (block_size == BIG_BLOCK) printf("Block is of size 6x6x6!\n");
    else if (block_size == MOUNTAIN) {
        printf("Mountain detected!\n");
        return EXIT_ERROR;
    }
    else if (block_size == NO_BLOCK) {
        printf("Error no size detected\n");
        return EXIT_ERROR;
    }


    // if (objectDetectionTwist(dist_sensor, 60, r_angle, 80) == EXIT_FAILURE) return EXIT_FAILURE;
    // locationPrint(*r_loc_x, *r_loc_y, *r_angle);


    if (objectAproach(dist_sensor, 50, r_loc_x, r_loc_y, r_angle) == EXIT_FAILURE) return EXIT_FAILURE;
    straigth(2, SPEED_STRAIGHT_MED, r_loc_x, r_loc_y, r_angle);
    locationPrint(*r_loc_x, *r_loc_y, *r_angle);
    

    int block_colour;
    if (tca9548a_switch_channel(mux, MUX_CHANNEL_COLOUR_SENSOR_0) == EXIT_FAILURE) return EXIT_FAILURE;
    block_colour = colourSensor(colour_sensor, INTEGRATION_TIME_MS);
    if (block_colour == BLACK) printf("Block is black!\n");
    else if (block_colour == WHITE) printf("Block is white!\n");
    else if (block_colour == RED) printf("Block is red!\n");
    else if (block_colour == BLUE) printf("Block is blue!\n");
    else if (block_colour == GREEN) printf("Block is green!\n");

    return EXIT_SUCCESS;
} /* int objectScan(vl53x* dist_sensor, tca9548a* mux, tcs3472* colour_sensor, double* r_loc_x, double* r_loc_y, double* r_angle) */



// =================== Advanced movement functions ================== //
int borderMovement(double* r_loc_x, double* r_loc_y, double* r_angle) {
    straigth(50, 30000, r_loc_x, r_loc_y, r_angle);
        
    while(checkIfDone()) {
        if (IRborderDetection() == IR_BOTH) {
            locationRecalculation(r_loc_x, r_loc_y, r_angle);
            printf("\033[33mIR Detects BOTH\033[0m\n\n");
            if (borderDetection(r_loc_x, r_loc_y, r_angle) == BORDER) return BORDER;
            else return CLIFF;
        }
        else if (IRborderDetection() == IR_LEFT) {
            printf("\033[33mIR Detects LEFT\033[0m\n\n");
            locationRecalculation(r_loc_x, r_loc_y, r_angle);
            turnLeft(90, 25000, r_loc_x, r_loc_y, r_angle);

            while(checkIfDone()) {
                if (IRborderDetection() == IR_BOTH) {
                    locationRecalculation(r_loc_x, r_loc_y, r_angle);

                    if (borderDetection(r_loc_x, r_loc_y, r_angle) == BORDER) return BORDER;
                    else return CLIFF;
                }
            }
        }
        else if (IRborderDetection() == IR_RIGHT) {
            printf("\033[33mIR Detects RIGHT\033[0m\n\n");
            locationRecalculation(r_loc_x, r_loc_y, r_angle);
            turnRight(90, 25000, r_loc_x, r_loc_y, r_angle);
            
            while(checkIfDone()) {
                if (IRborderDetection() == IR_BOTH) {
                    locationRecalculation(r_loc_x, r_loc_y, r_angle);
                    
                    if (borderDetection(r_loc_x, r_loc_y, r_angle) == BORDER) return BORDER;
                    else return CLIFF;
                }
            }
        }
    }
    return EXIT_ERROR;
}

int borderDetection(double* r_loc_x, double* r_loc_y, double* r_angle) {
    locationRecalculation(r_loc_x, r_loc_y, r_angle);

    straigth(4, SPEED_STRAIGHT_MED, r_loc_x, r_loc_y, r_angle);
    waitTillDone();
    locationRecalculation(r_loc_x, r_loc_y, r_angle);

    if (IRborderDetection() == IR_NONE) {
        avoidBorder(r_loc_x, r_loc_y, r_angle);
        locationRecalculation(r_loc_x, r_loc_y, r_angle);
        return BORDER;
    } else {
        double c_loc_x = *r_loc_x + 15 * cosin(*r_angle);
        double c_loc_y = *r_loc_y + 15 * sinus(*r_angle);

        printf("\033[34m\nCliff Location:\033[0m (%.1f, %.1f)\n\n", c_loc_x, c_loc_y);
        //
        // Send info to server
        //

        avoidCliff(r_loc_x, r_loc_y, r_angle);
        locationRecalculation(r_loc_x, r_loc_y, r_angle);
        return CLIFF;
    }
}
