/* QR Code decoding using quirc library
 * Source: https://github.com/dlbeer/quirc
 * Used for QR code detection in box sorting system
 */

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "quirc/quirc.h"

#define TIME_STEP     32
#define COOLDOWN_MS   4000  // 4 seconds after robot finishes

// box spawn position - start of conveyor
#define SPAWN_X -2.9
#define SPAWN_Y 7.96
#define SPAWN_Z 0.42

// supervisor handle to youbot node and its customData field
static WbNodeRef youbot_node = NULL;
static WbFieldRef youbot_custom_field = NULL;

// toggle infinite respawn
bool infinite = true;

// teleports a box at random to the start of the conveyor
void respawn_box() {
  if (!infinite)
    return;

  int choice = rand() % 2;
  const char *box_name = (choice == 0) ? "box_A" : "box_B";

  WbNodeRef box = wb_supervisor_node_get_from_def(box_name);
  if (!box) {
    printf("Error: Could not find %s\n", box_name);
    return;
  }

  WbFieldRef translation = wb_supervisor_node_get_field(box, "translation");
  WbFieldRef rotation = wb_supervisor_node_get_field(box, "rotation");
  const double new_pos[3] = {SPAWN_X, SPAWN_Y, SPAWN_Z};
  const double new_rot[4] = {0, 1, 0, 0};
  wb_supervisor_field_set_sf_vec3f(translation, new_pos);
  wb_supervisor_field_set_sf_rotation(rotation, new_rot);
  printf("respawned %s\n", box_name);
}

// informs the youbot controller of decoded qr
static void send_box_category(const char *category) {
  if (!youbot_node || !youbot_custom_field)
    return;

  wb_supervisor_field_set_sf_string(youbot_custom_field, category);
  printf("Camera: set youbot customData = '%s'\n", category);
}

// Camera controller state machine
typedef enum {
  CAM_WAIT_BOX = 0,   // waiting to see a QR and send CHUTE_*
  CAM_WAIT_ROBOT,     // QR sent, waiting for robot to finish
  CAM_COOLDOWN        // robot finished; wait 4s then spawn next box
} CamState;

int main() {
  wb_robot_init();
  srand(time(NULL));

  // get youbot node and its customData field
  youbot_node = wb_supervisor_node_get_from_def("ROBOT");
  if (!youbot_node)
    printf("Camera: WARNING: DEF ROBOT not found\n");
  else {
    youbot_custom_field = wb_supervisor_node_get_field(youbot_node, "customData");
    if (!youbot_custom_field)
      printf("Camera: WARNING: ROBOT has no customData field\n");
    else
      wb_supervisor_field_set_sf_string(youbot_custom_field, "");  // clear on world load
  }

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  struct quirc *qr = quirc_new();
  if (!qr) {
    printf("Failed to create quirc object\n");
    return 1;
  }

  printf("Camera controller started with QR code detection.\n");

  // simple state machine
  CamState state = CAM_WAIT_BOX;
  int cooldown_timer = 0;  // in ms

  // Spawn initial box so there is something to scan
  respawn_box();

  while (wb_robot_step(TIME_STEP) != -1) {

    switch (state) {
      case CAM_WAIT_BOX: {
        const unsigned char *image = wb_camera_get_image(camera);
        if (!image)
          break;

        // prepare quirc image
        int width = wb_camera_get_width(camera);
        int height = wb_camera_get_height(camera);

        if (quirc_resize(qr, width, height) < 0) {
          printf("Failed to resize quirc\n");
          break;
        }

        unsigned char *qr_image = quirc_begin(qr, NULL, NULL);
        for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
            int idx = (y * width + x) * 4;
            unsigned char gray = (image[idx] + image[idx + 1] + image[idx + 2]) / 3;
            qr_image[y * width + x] = gray;
          }
        }
        quirc_end(qr);

        int num_codes = quirc_count(qr);
        for (int i = 0; i < num_codes; i++) {
          struct quirc_code code;
          struct quirc_data data;

          quirc_extract(qr, i, &code);
          quirc_decode_error_t err = quirc_decode(&code, &data);

          if (err == QUIRC_SUCCESS) {
            printf("Decoded QR Code: %s\n", data.payload);

            if (strcmp((char *)data.payload, "CHUTE_A") == 0) {
              printf("Camera: QR → CHUTE_A\n");
              send_box_category("CHUTE_A");
              state = CAM_WAIT_ROBOT;
              break;
            } else if (strcmp((char *)data.payload, "CHUTE_B") == 0) {
              printf("Camera: QR → CHUTE_B\n");
              send_box_category("CHUTE_B");
              state = CAM_WAIT_ROBOT;
              break;
            } else {
              printf("Camera: Unknown QR '%s'\n", data.payload);
            }
          } else {
            printf("QR decode error: %s\n", quirc_strerror(err));
          }
        }
        break;
      }

      case CAM_WAIT_ROBOT: {
        // Robot will set customData = "DONE" when it has dropped off the box.
        if (youbot_custom_field) {
          const char *cd = wb_supervisor_field_get_sf_string(youbot_custom_field);
          if (cd && strcmp(cd, "DONE") == 0) {
            printf("Camera: robot signalled DONE → start 4s cooldown\n");
            // clear field again
            wb_supervisor_field_set_sf_string(youbot_custom_field, "");
            cooldown_timer = COOLDOWN_MS;
            state = CAM_COOLDOWN;
          }
        }
        break;
      }

      case CAM_COOLDOWN: {
        if (cooldown_timer > 0) {
          cooldown_timer -= TIME_STEP;
          if (cooldown_timer <= 0) {
            printf("Camera: cooldown finished → spawn next box\n");
            respawn_box();
            state = CAM_WAIT_BOX;
          }
        }
        break;
      }

      default:
        state = CAM_WAIT_BOX;
        break;
    }
  }

  quirc_destroy(qr);
  wb_robot_cleanup();
  return 0;
}
