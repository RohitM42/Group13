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

#define TIME_STEP 32
#define ROBOT_RADIUS 0.32

// Destination for category CHUTE_A
#define DEST_A_X   0.2
#define DEST_A_Y   4.23
#define DEST_A_YAW 0.0  // facing chute

// Destination for category CHUTE_B
#define DEST_B_X   -1.91
#define DEST_B_Y   4.91
#define DEST_B_YAW 1.57

// Destination for conveyor
#define DEST_C_X   -0.757
#define DEST_C_Y   7.42
#define DEST_C_YAW 1.57

// Webots Web Interface receive function (available to controllers)
const char *wb_robot_wwi_receive_text(void);

// remember last customData so we only react to changes
static char last_category[32] = "";

// current box we're processing
static char active_category[32] = "";

// simple state machine for the full "box handling" sequence
typedef enum {
  TASK_IDLE = 0,
  TASK_GO_TO_CONVEYOR,
  TASK_WAIT_BEFORE_PICK,
  TASK_WAIT_AFTER_PICK,
  TASK_GO_TO_CHUTE,
  TASK_WAIT_AFTER_DROPOFF
} TaskState;

static TaskState task_state = TASK_IDLE;
static double wait_until = 0.0;  // simulation time to wait until

static void process_window_messages() {
  const char *msg;
  while ((msg = wb_robot_wwi_receive_text()) != NULL) {
    printf("[WINDOW] Raw message: '%s'\n", msg);

    double x, y, yaw;

    if (sscanf(msg, "NAV_GOTO %lf %lf %lf", &x, &y, &yaw) == 3) {
      printf("[WINDOW] Parsed NAV_GOTO: x=%.3f, y=%.3f, yaw=%.3f\n",
             x, y, yaw);
      nav_start_to(x, y, yaw);
    }
    else if (sscanf(msg, "GOTO %lf %lf %lf", &x, &y, &yaw) == 3) {
      printf("[WINDOW] Parsed GOTO (using planner): x=%.3f, y=%.3f, yaw=%.3f\n",
             x, y, yaw);
      nav_start_to(x, y, yaw);
    }
    // Arm commands from robot window (for testing)
    else if (strcmp(msg, "ARM_RESET") == 0) {
      printf("[WINDOW] ARM_RESET\n");
      arm_reset();
    }
    else if (strcmp(msg, "ARM_PICK") == 0) {
      printf("[WINDOW] ARM_PICK\n");
      arm_pick();
    }
    else if (strcmp(msg, "ARM_HOLD") == 0) {
      printf("[WINDOW] ARM_HOLD\n");
      arm_hold();
    }
    else if (strcmp(msg, "ARM_DROPOFF") == 0) {
      printf("[WINDOW] ARM_DROPOFF\n");
      arm_dropoff();
    }
    else if (strcmp(msg, "ARM_HEIGHT_UP") == 0) {
      printf("[WINDOW] ARM_HEIGHT_UP\n");
      arm_increase_height();
    }
    else if (strcmp(msg, "ARM_HEIGHT_DOWN") == 0) {
      printf("[WINDOW] ARM_HEIGHT_DOWN\n");
      arm_decrease_height();
    }
    else if (strcmp(msg, "ARM_ORIENT_INC") == 0) {
      printf("[WINDOW] ARM_ORIENT_INC\n");
      arm_increase_orientation();
    }
    else if (strcmp(msg, "ARM_ORIENT_DEC") == 0) {
      printf("[WINDOW] ARM_ORIENT_DEC\n");
      arm_decrease_orientation();
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

  // only react to changes
  if (strcmp(data, last_category) == 0)
    return;

  strncpy(last_category, data, sizeof(last_category) - 1);
  last_category[sizeof(last_category) - 1] = '\0';

  printf("[MSG] customData from supervisor: '%s'\n", data);

  // Only start a new job if we are idle
  if (task_state != TASK_IDLE) {
    printf("[MSG] Robot busy (state=%d), ignoring '%s'\n",
           (int)task_state, data);
    return;
  }

  if (strcmp(data, "CHUTE_A") == 0 || strcmp(data, "CHUTE_B") == 0) {
    strncpy(active_category, data, sizeof(active_category) - 1);
    active_category[sizeof(active_category) - 1] = '\0';

    printf("[TASK] New box '%s' → first go to conveyor\n", active_category);
    nav_start_to(DEST_C_X, DEST_C_Y, DEST_C_YAW);
    task_state = TASK_GO_TO_CONVEYOR;
  } else {
    printf("[MSG] Unknown category '%s'\n", data);
  }
}

static void update_task_state(void) {
  double now = wb_robot_get_time();
  switch (task_state) {
    case TASK_IDLE:
      // nothing to do
      break;
    case TASK_GO_TO_CONVEYOR:
      // wait until navigation is finished
      if (!nav_is_active() && base_goto_reached()) {
        printf("[TASK] Arrived at conveyor. Waiting 2s before pick pose.\n");
        wait_until = now + 2.0;
        task_state = TASK_WAIT_BEFORE_PICK;
      }
      break;
    case TASK_WAIT_BEFORE_PICK:
      if (now >= wait_until) {
        printf("[TASK] Calling arm_pick() (this includes its own internal wait).\n");
        arm_pick();  // this already waits ARM_DELAY inside
        // after pick pose, wait 2s before going to hold pose
        wait_until = wb_robot_get_time() + 2.7;
        task_state = TASK_WAIT_AFTER_PICK;
      }
      break;
    case TASK_WAIT_AFTER_PICK:
      if (now >= wait_until) {
        printf("[TASK] Calling arm_hold() (this includes its own internal wait).\n");
        arm_hold();  // also includes its own ARM_DELAY
        // now move to the correct chute
        if (strcmp(active_category, "CHUTE_A") == 0) {
          printf("[TASK] Moving to CHUTE_A\n");
          nav_start_to(DEST_A_X, DEST_A_Y, DEST_A_YAW);
        } else if (strcmp(active_category, "CHUTE_B") == 0) {
          printf("[TASK] Moving to CHUTE_B\n");
          nav_start_to(DEST_B_X, DEST_B_Y, DEST_B_YAW);
        } else {
          printf("[TASK] Unknown active_category '%s', aborting task.\n",
                 active_category);
          active_category[0] = '\0';
          task_state = TASK_IDLE;
          break;
        }
        task_state = TASK_GO_TO_CHUTE;
      }
      break;
    case TASK_GO_TO_CHUTE:
      if (!nav_is_active() && base_goto_reached()) {
        printf("[TASK] Arrived at chute. Dropping off box.\n");
        arm_dropoff();  // includes its own ARM_DELAY inside
        printf("[TASK] Waiting 2s after dropoff before arm reset.\n");
        wait_until = wb_robot_get_time() + 2.0;
        task_state = TASK_WAIT_AFTER_DROPOFF;
      }
      break;
    case TASK_WAIT_AFTER_DROPOFF:
      if (now >= wait_until) {
        printf("[TASK] Resetting arm.\n");
        arm_reset();

        // Clear active and last category
        active_category[0] = '\0';
        last_category[0]   = '\0';

        // Signal to camera that this box has been delivered.
        wb_robot_set_custom_data("DONE");

        task_state = TASK_IDLE;
        printf("[TASK] Cycle finished – waiting for next box.\n");
      }
      break;
    default:
      task_state = TASK_IDLE;
      break;
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

  obstacles_register_rect_from_def("CONVEYOR",  ROBOT_RADIUS);
  obstacles_register_rects_by_type("RoCKInShelf", ROBOT_RADIUS);
  obstacles_register_rects_by_type("WoodenPalletStack", ROBOT_RADIUS);

  while (wb_robot_step(TIME_STEP) != -1) {
    // UI + new box detection
    process_window_messages();
    process_custom_data_message();

    // low-level base goto
    if (!base_goto_reached())
      base_goto_run();
    else
      base_reset();

    // path following
    nav_update();

    // high-level task sequencing (conveyor → pick/hold → chute → drop/reset)
    update_task_state();
  }

  base_reset();
  wb_robot_cleanup();
  return 0;
}
