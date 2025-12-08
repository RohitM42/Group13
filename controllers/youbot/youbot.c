#include <webots/robot.h>

// Webots Web Interface receive function (available to controllers)
const char *wb_robot_wwi_receive_text(void);

#include <arm.h>
#include <base.h>
#include <gripper.h>
#include "obstacles.h"
#include "navigation.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32
#define ROBOT_RADIUS 0.35

static void process_window_messages() {
  const char *msg;
  while ((msg = wb_robot_wwi_receive_text()) != NULL) {
    printf("[WINDOW] Raw message: '%s'\n", msg);

    double x, y, yaw;

    if (sscanf(msg, "NAV_GOTO %lf %lf %lf", &x, &y, &yaw) == 3) {
      printf("[WINDOW] Parsed NAV_GOTO: x=%.3f, y=%.3f, yaw=%.3f\n", x, y, yaw);
      nav_start_to(x, y, yaw);
    }
    else if (sscanf(msg, "GOTO %lf %lf %lf", &x, &y, &yaw) == 3) {
      printf("[WINDOW] Parsed GOTO (using planner): x=%.3f, y=%.3f, yaw=%.3f\n", x, y, yaw);
      nav_start_to(x, y, yaw);
    }
    else {
      printf("[WINDOW] Unknown message format\n");
    }
  }
}

int main(int argc, char **argv) {
  wb_robot_init();

  base_init();
  arm_init();
  gripper_init();
  base_goto_init();
  nav_init();

  obstacles_init();

  // obstacles_register_rect_from_def("CONVEYOR",  ROBOT_RADIUS);
  // obstacles_register_rect_from_def("PALLET_1",  ROBOT_RADIUS);
  // obstacles_register_rect_from_def("PALLET_2",  ROBOT_RADIUS);
  obstacles_register_rects_by_type("WoodenPallet", ROBOT_RADIUS);

  while (wb_robot_step(TIME_STEP) != -1) {
    process_window_messages();

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
