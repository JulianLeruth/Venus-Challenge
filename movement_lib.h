#define PI                 3.14159265358979323846
#define ROBOT_WIDTH                          12.3
#define STRAIGTH_STEPS_IN_CM                   64
#define NUMBER_OF_MEASUREMENT_FOR_TURNING_360 120
#define NUMBER_OF_DIST_MEASUREMENT_PER_SECOND  20

#define SMALL_BLOCK_MIN_NUM_MEASUREMENTS        5          // The minimum number of measurements it should take for the robot to detect a small block.
#define SMALL_BLOCK_MAX_NUM_MEASUREMENTS       15          // The maximum number of measurements it can take for the robot to detect a small block and the minimum for the robot to detect a big block.
#define BIG_BLOCK_MAX_NUM_MEASUREMENTS         25          // The maximum number of measurements it can take for the robot to detect a big block.

#define SMALL_BLOCK                             2
#define BIG_BLOCK                               3
#define MOUNTAIN                                4
#define NO_BLOCK                                5

#define MUX_CHANNEL_DIST_SENSOR_0               0
#define MUX_CHANNEL_DIST_SENSOR_1               1
#define MUX_CHANNEL_COLOUR_SENSOR_0             7
#define MUX_CHANNEL_COLOUR_SENSOR_1             3

#define INTEGRATION_TIME_MS                    60

#define BLACK                                   0
#define WHITE                                   1
#define RED                                     2
#define BLUE                                    3
#define GREEN                                   4

#define EXIT_ERROR                              2
// ===================== Basis Value Functions ====================== //
/**
 * @brief The 'sizeOfArray' function calculates the size of the given arr.
 * 
 * @param arr       An array.
 * 
 * @returns The size of the given array.
*/
extern int sizeOfArray(int* arr);


/**
 * @brief The 'highestValueIndex' function finds the highest value of the given array.
 * 
 * @param arr       An array.
 * 
 * @returns The index of the highest value in the given array.
*/
extern int highestValueIndex(int* arr);

/**
 * @brief The 'lowestValueIndex' function finds the lowest value of the given array.
 * 
 * @param arr       An array.
 * 
 * @returns The index of the lowest value in the given array or -1 incase of non-low value.
*/
extern int lowestValueIndex(int* arr);

/**
 * @brief The 'convoluteArr' function takes an average of the value before, the value of and the value after an index of the array.
 * 
 * @param arr       An array.
*/
extern void convoluteArr(int* arr);

/**
 * @brief The 'cosin' function calculates the cosinus value of an angle in degrees.
 * 
 * @param angle     The angle with which the cosinus is calculated.
 * 
 * @returns The cosinus value of the given angle.
*/
extern double cosin(double angle);

/**
 * @brief The 'sinus' function calculates the sinus value of an angle in degrees.
 * 
 * @param angle     The angle with which the sinus is calculated.
 * 
 * @returns The sinus value of the given angle.
*/
extern double sinus(double angle);


// ===================== Localization Functions ===================== //

/**
 * @brief The 'blockLocation' function calculates the location of the block
 *          based on the location of the robot and the distance between the two.
 * 
 * @param r_loc_x   The X location of the robot.
 * @param r_loc_y   The Y location of the robot.
 * @param r_angle   The angle of the robot.
 * @param dist      The angle of the robot.
 * @param b_loc_x   The X location of the block.
 * @param b_loc_y   The Y location of the block.
 * 
*/
extern void blockLocation(double r_loc_x, double r_loc_y, double r_angle, int32_t dist, double* b_loc_x, double* b_loc_y);

/**
 * @brief The 'location' function prints the currect location of the robot.
 * 
 * @param r_loc_x   The X location of the robot.
 * @param r_loc_y   The Y location of the robot.
 * @param r_angle   The angle of the robot.
*/
extern void location(double r_loc_x, double r_loc_y, double r_angle);


// ==================== Movement check functions ==================== //
/**
 * @brief The 'waitTillDone' function stops the code until
 *          all steps are taken by the stepper motor.
*/
extern void waitTillDone(void);

/**
 * @brief The 'checkIfDone' function check if all steps
 *          are taken by the stepper motor.
 * 
 * @returns 0 if all steps are taken, 1 if the motor is not done yet.
*/
extern int check_if_done(void);


// ==================== Basic Movement Funtions ===================== //
/**
 * @brief The 'straight' function makes the robot go straight
 *          for a given amount of distance and a given speed.
 * 
 * @param dist      The distance the robot makes.
 * @param speed     The speed of the robot.
 * @param r_loc_x   The X location of the robot.
 * @param r_loc_y   The Y location of the robot.
 * @param r_angle   The angle of the robot.
*/
extern void straigth(int dist, uint16_t speed, double* r_loc_x, double* r_loc_y, double* r_angle);

/**
 * @brief The 'twist' function makes the robot turn around it's own axes
 *          for a given amount of degrees and a given speed.
 * 
 * @param angle     The angle of the turn.
 * @param speed     The speed of the turn.
 * @param r_angle   The angle of the robot.
*/
extern void twist(int angle, uint16_t speed, double* r_angle);



// =========== Advanced Movement and detection functions ============ //
/**
 * @brief The 'sizeDetection' function measures the size of an object.
 * 
 * @param sensor    Handle to the VL53L0X distance sensor.
 * @param r_angle   The angle of the robot.
 * 
 * @returns EXIT_SUCCESS [0] on success, EXIT_FAILURE [1] on failure.
*/
extern int sizeDetection(vl53x* sensor, double* r_angle);

/**
 * @brief The 'objectDetectionTwist' function make turn and
 *          detects the closest objects to the robot
 *          and then turns back to the closest object.
 * 
 * @param sensor    Handle to the VL53L0X distance sensor.
 * @param angle     The angle the robot will turn to search.
 * @param r_angle   The angle of the robot.
 * @param max_dist  The distance until which the sensor detects.
 * 
 * @returns EXIT_SUCCESS [0] on success, EXIT_FAILURE [1] on failure, EXIT_ERROR [2] on error.
*/
extern int objectDetectionTwist(vl53x* sensor, int angle, double* r_angle, int max_dist);

/**
 * @brief The 'objectAproach' function aproaches an object
 *          and stops if the object is closer than the a given distance.
 * 
 * @param sensor    Handle to the VL53L0X distance sensor.
 * @param max_dist  The distance the sensor needs to approuch.
 * @param r_loc_x   The X location of the robot.
 * @param r_loc_y   The Y location of the robot.
 * @param r_angle   The angle of the robot.
 * 
 * @returns EXIT_SUCCESS [0] on success, EXIT_FAILURE [1] on failure.
*/
extern int objectAproach(vl53x* sensor, int max_dist, double* r_loc_x, double* r_loc_y, double* r_angle);

/**
 * @brief The 'objectAproach' function aproaches an object
 *          and stops if the object is closer than the a given distance.
 * 
 * @param dist_sensor   Handle to the VL53L0X distance sensor.
 * @param mux           Handle to the TCA9548A multiplexer.
 * @param colour_sensor Handle to the TCS3472 colour_sensor.
 * @param r_loc_x       The X location of the robot.
 * @param r_loc_y       The Y location of the robot.
 * @param r_angle       The angle of the robot.
 * 
 * @returns EXIT_SUCCESS [0] on success, EXIT_FAILURE [1] on failure, EXIT_ERROR [2] on error.
*/
extern int objectScan(vl53x* dist_sensor, tca9548a* mux, tcs3472* colour_sensor, double* r_loc_x, double* r_loc_y, double* r_angle);
