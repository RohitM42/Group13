#include <webots/robot.h>
#include <webots/receiver.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>
#include "obstacles.h"
#include "navigation.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// for when the camera sends messages
// #include <webots/receiver.h>

// temporary 
// Destination for category CHUTE_A
#define DEST_A_X   0.2
#define DEST_A_Y    4.23
#define DEST_A_YAW (0.0)  // facing chute

// Destination for category CHUTE_B
#define DEST_B_X   -1.91
#define DEST_B_Y    4.91
#define DEST_B_YAW (1.57)

// Destination for conveyor
#define DEST_C_X   -0.784
#define DEST_C_Y    7.36
#define DEST_C_YAW (1.57)

// Webots Web Interface receive function (available to controllers)
const char *wb_robot_wwi_receive_text(void);

#define TIME_STEP 32
#define ROBOT_RADIUS 0.32
static char last_category[32] = "";

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

static void process_custom_data_message() {
  const char *data = wb_robot_get_custom_data();
  if (!data || !data[0])
    return;

  if (strcmp(data, last_category) == 0)
    return;

  strncpy(last_category, data, sizeof(last_category) - 1);
  last_category[sizeof(last_category) - 1] = '\0';

  printf("[MSG] customData from supervisor: '%s'\n", data);

  if (nav_is_active()) {
    printf("[MSG] Robot busy, ignoring '%s'\n", data);
    return;
  }

  if (strcmp(data, "CHUTE_A") == 0) {
    printf("[MSG] → navigate to DEST A\n");
    nav_start_to(DEST_A_X, DEST_A_Y, DEST_A_YAW);
  } else if (strcmp(data, "CHUTE_B") == 0) {
    printf("[MSG] → navigate to DEST B\n");
    nav_start_to(DEST_B_X, DEST_B_Y, DEST_B_YAW);
  } else {
    printf("[MSG] Unknown category '%s'\n", data);
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

  // obstacles_register_rect_from_def("PALLET_1",  ROBOT_RADIUS);
  // obstacles_register_rect_from_def("PALLET_2",  ROBOT_RADIUS);
  obstacles_register_rect_from_def("CONVEYOR",  ROBOT_RADIUS);
  obstacles_register_rects_by_type("RoCKInShelf", ROBOT_RADIUS);
  obstacles_register_rects_by_type("WoodenPalletStack", ROBOT_RADIUS);

  while (wb_robot_step(TIME_STEP) != -1) {
    process_window_messages();
    process_custom_data_message();

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
