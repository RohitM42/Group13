#include <webots/keyboard.h>
#include <webots/robot.h>

// Webots Web Interface receive function (available to controllers)
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

static void process_window_messages() {
  const char *msg;
  while ((msg = wb_robot_wwi_receive_text()) != NULL) {
    printf("[WINDOW] Raw message: '%s'\n", msg);

    double x, y, yaw;
    // GOTO x y yaw
    if (sscanf(msg, "GOTO %lf %lf %lf", &x, &y, &yaw) == 3) {
      printf("[WINDOW] Parsed GOTO: x=%.3f, y=%.3f, yaw=%.3f\n", x, y, yaw);
      base_goto_set_target(x, y, yaw);
    }
    // NAV_CONVEYOR_P1  (conveyor -> pallet 1)
    else if (strncmp(msg, "NAV_CONVEYOR_P1", 15) == 0) {
      printf("[WINDOW] NAV_CONVEYOR_P1\n");
      nav_start_from_conveyor_to(PALLET_1);
    }
    // NAV_CONVEYOR_P2  (conveyor -> pallet 2)
    else if (strncmp(msg, "NAV_CONVEYOR_P2", 15) == 0) {
      printf("[WINDOW] NAV_CONVEYOR_P2\n");
      nav_start_from_conveyor_to(PALLET_2);
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

  while (wb_robot_step(TIME_STEP) != -1) {
    // 1) Handle commands from robot window
    process_window_messages();

    // 2) Run low-level goto
    if (!base_goto_reached())
      base_goto_run();
    else
      base_reset();  // stop wheels when we reached the current target

    // 3) Let navigation advance to next waypoint when needed
    nav_update();
  }

  base_reset();
  wb_robot_cleanup();
  return 0;
}
