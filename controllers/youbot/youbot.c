#include <webots/keyboard.h>
#include <webots/robot.h>

const char *wb_robot_wwi_receive_text(void);

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// navigation
#include "navigation.h"

#define TIME_STEP 32

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void process_window_messages() {
  const char *msg;
  while ((msg = wb_robot_wwi_receive_text()) != NULL) {
    printf("[WINDOW] Raw message: '%s'\n", msg);
    double x, y, yaw;
    if (sscanf(msg, "GOTO %lf %lf %lf", &x, &y, &yaw) == 3) {
      printf("[WINDOW] Parsed goto: x=%.3f, y=%.3f, yaw=%.3f\n", x, y, yaw);
      base_goto_set_target(x, y, yaw);
    } else {
      printf("[WINDOW] Unknown message format\n");
    }
  }
}

// int main(int argc, char **argv) {
//   wb_robot_init();

//   base_init();
//   arm_init();
//   gripper_init();
//   base_goto_init();

//   while (wb_robot_step(TIME_STEP) != -1) {
//     // navigation web
//     process_window_messages();
//     step();

//     // run until reached target
//     if (!base_goto_reached())
//       base_goto_run();
//     else
//       base_reset();  // stop wheels when we reached the target
//   }

//   base_reset();
//   wb_robot_cleanup();
//   return 0;
// }

// test
int main(int argc, char **argv) {
  wb_robot_init();

  base_init();
  arm_init();
  gripper_init();
  base_goto_init();
  nav_init();

  nav_start_from_conveyor_to(PALLET_1);

  while (wb_robot_step(TIME_STEP) != -1) {

    if (!base_goto_reached())
      base_goto_run();
    else
      base_reset();

    nav_update();
  }

  base_reset();
  wb_robot_cleanup();
  return 0;
}
