#include <libpynq.h>
#include <stepper.h>
#include <math.h>
#include "movement_lib.h"

int main(void) {
  /* == Initiation == */
  pynq_init();
  stepper_init();
  stepper_reset();

  // dance();
  // straigth(10);
  // turn_right(90);
  // turn_right(-90);
  // turn_left(90);
  // turn_left(-90);
  // twist(360*5);
  // sleep_msec(5000);
  // twist(-360*5);
  strave_right(90,10);
  strave_left(90,10);
  
  /* == Clean-up == */
  stepper_destroy();
  pynq_destroy();
  return EXIT_SUCCESS;
}
