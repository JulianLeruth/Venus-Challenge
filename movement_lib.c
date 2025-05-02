#include <libpynq.h>
#include <stepper.h>
#include <math.h>
/*
  *=== Motor Size, Speed and Steps Info ===*
        Minimum speed = 3024
        This gives 30 microsec per step
            and 50 millisec per rotation
        Maximum speed = 65535
        This gives 655 microsec per step
            and 1 sec per rotation
        Wheel have diameter of ~8 cm and circonfrence of ~25 cm
        1 Rotation ~= 1600 Steps
        1 cm ~= 64 steps
*/

#define PI 3.14159265358979323846
#define STRAIGTH_STEPS_IN_CM 64     // Max number of steps ~= 32767
#define TURN_STEPS_IN_HALF_CM 32    // Max number of steps ~= 32767
#define MOVEMENT_SPEED 15000        // Defined on trial and error
#define ROTATION_SPEED 15000        // Defined on trial and error
#define ROBOT_WIDTH 12.3


void wait_till_done(void){
  int16_t TEMP_right = 0;
  int16_t TEMP_left = 0;
  do {
    stepper_get_steps(&TEMP_right, &TEMP_left);
    sleep_msec(50);
  } while(TEMP_right!=0 || TEMP_left!=0);
  return;
}

void straigth(int dist /* Defined in cm */){
  stepper_enable();
  stepper_set_speed(MOVEMENT_SPEED, MOVEMENT_SPEED);

  stepper_steps(STRAIGTH_STEPS_IN_CM*dist, STRAIGTH_STEPS_IN_CM*dist);
  wait_till_done();

  stepper_disable();
  stepper_reset();
  sleep_msec(500);
}

void turn_right(int angle /* Defined in degrees */){
  float number_of_steps = (ROBOT_WIDTH*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  stepper_enable();
  stepper_set_speed(ROTATION_SPEED, ROTATION_SPEED);
  
  printf("Float: %.1f, Int: %d", number_of_steps, (int)(number_of_steps));

  stepper_steps(0, (int)(number_of_steps));
  wait_till_done();
  
  stepper_disable();
  stepper_reset();
  sleep_msec(100);
}

void turn_left(int angle /* Defined in degrees */){
  float number_of_steps = (ROBOT_WIDTH*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  stepper_enable();
  stepper_set_speed(ROTATION_SPEED, ROTATION_SPEED);
  
  stepper_steps((int)(number_of_steps), 0);
  wait_till_done();
  
  stepper_disable();
  stepper_reset();
  sleep_msec(100);
}

void twist(int angle /* Defined in degrees */){
  float number_of_steps = (ROBOT_WIDTH*PI*angle*STRAIGTH_STEPS_IN_CM)/360;

  stepper_enable();
  stepper_set_speed(ROTATION_SPEED, ROTATION_SPEED);

  stepper_steps(-(int)(number_of_steps), (int)(number_of_steps));
  wait_till_done();
  
  stepper_disable();
  stepper_reset();
  sleep_msec(100);
}

void strave_right(int angle, int dist /* Defined in degrees */){
  float number_of_steps_left = ((ROBOT_WIDTH + dist)*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  float number_of_steps_right = (dist*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  float rotation_speed_left = ROTATION_SPEED;
  float rotation_speed_right = (ROBOT_WIDTH + dist)/dist*rotation_speed_left;

  stepper_enable();
  stepper_set_speed(rotation_speed_left, rotation_speed_right);
  
  stepper_steps((int)(number_of_steps_left), (int)(number_of_steps_right));
  wait_till_done();
  
  stepper_disable();
  stepper_reset();
  sleep_msec(100);
}

void strave_left(int angle, int dist /* Defined in degrees */){
  float number_of_steps_right = ((ROBOT_WIDTH + dist)*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  float number_of_steps_left = (dist*2*PI*angle*STRAIGTH_STEPS_IN_CM)/360;
  float rotation_speed_right = ROTATION_SPEED;
  float rotation_speed_left = (ROBOT_WIDTH + dist)/dist*rotation_speed_right;

  stepper_enable();
  stepper_set_speed(rotation_speed_left, rotation_speed_right);

  stepper_steps((int)(number_of_steps_left), (int)(number_of_steps_right));
  wait_till_done();

  stepper_disable();
  stepper_reset();
  sleep_msec(100);
}

void dance(void){
  turn_right(18);
  for(int i = 0; i < 5; i++){
    turn_left(30);
    turn_right(30);
  }
  turn_left(18);
  twist(-90);
  straigth(25);
  turn_right(90);
  twist(180);
  strave_left(30,10);
  for(int i = 0; i < 5; i++){
    strave_right(-60,10);
    strave_left(60,10);
  }
  strave_right(-30,10);
}
