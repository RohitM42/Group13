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

#define TIME_STEP 32
#define COOLDOWN 20000 //20 seconds
#define RESPAWN_TIME 4000 //4 seconds

// needs to be adjusted to the current environment
#define SPAWN_X -2.9
#define SPAWN_Y 7.96
#define SPAWN_Z 0.35

// for emitter to send messages to youbot controller
static WbNodeRef youbot_node = NULL;
static WbFieldRef youbot_custom_field = NULL;

bool infinite = true;

void respawn_box(){
  if (!infinite){
    return;
  }

  int choice = rand() % 2;
  const char* box_name = (choice == 0) ? "box_A" : "box_B";

  WbNodeRef box = wb_supervisor_node_get_from_def(box_name);
  if (!box){
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

static void send_box_category(const char *category) {
  if (!youbot_node || !youbot_custom_field)
    return;

  wb_supervisor_field_set_sf_string(youbot_custom_field, category);
  printf("Camera: set youbot customData = '%s'\n", category);
}

int main() {
  wb_robot_init();
  srand(time(NULL));

  youbot_node = wb_supervisor_node_get_from_def("ROBOT");
  if (!youbot_node)
    printf("Camera: WARNING: DEF ROBOT not found\n");
  else {
    youbot_custom_field = wb_supervisor_node_get_field(youbot_node, "customData");
    if (!youbot_custom_field)
      printf("Camera: WARNING: ROBOT has no customData field\n");
    else // reset the value when the world reloads
      wb_supervisor_field_set_sf_string(youbot_custom_field, "");
  }

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  
  struct quirc *qr = quirc_new();
  if (!qr) {
    printf("Failed to create quirc object\n");
    return 1;
  }

  printf("Camera controller started with QR code detection.\n");

  int cooldown_timer = 0;
  int respawn_timer = RESPAWN_TIME;
  bool code_detected = false;

  while (wb_robot_step(TIME_STEP) != -1) {

    if (cooldown_timer > 0) {
      cooldown_timer -= TIME_STEP;
      continue;
    }

    if (respawn_timer > 0) {
      respawn_timer -= TIME_STEP;
      if (respawn_timer <= 0) {
        respawn_box();
      }
    }

    const unsigned char *image = wb_camera_get_image(camera);
    
    if (image && !code_detected) {
      int width = wb_camera_get_width(camera);
      int height = wb_camera_get_height(camera);
      
      if (quirc_resize(qr, width, height) < 0) {
        printf("Failed to resize quirc\n");
        continue;
      }
      
      unsigned char *qr_image = quirc_begin(qr, NULL, NULL);
      
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          int idx = (y * width + x) * 4;
          unsigned char gray = (image[idx] + image[idx+1] + image[idx+2]) / 3;
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
          
          if (strcmp((char*)data.payload, "CHUTE_A") == 0) {
            printf("RED\n");
            send_box_category("CHUTE_A");
            cooldown_timer= COOLDOWN;
            respawn_timer = RESPAWN_TIME;
            code_detected = true;
            break;
          } else if (strcmp((char*)data.payload, "CHUTE_B") == 0) {
            printf("BLUE\n");
            send_box_category("CHUTE_B");
            cooldown_timer= COOLDOWN;
            respawn_timer = RESPAWN_TIME;
            code_detected = true;
            break;
          } else {
            printf("Unknown  %s\n", data.payload);
            send_box_category((char*)data.payload);
            cooldown_timer= COOLDOWN;
            respawn_timer = RESPAWN_TIME;
            code_detected = true;
            break;
          }
        } else {
          printf("QR decode error: %s\n", quirc_strerror(err));
        }
      }
    }
    if (cooldown_timer <= 0 && code_detected) {
      code_detected = false;
      printf("Cooldown ended.\n");
    }
  }

  quirc_destroy(qr);
  wb_robot_cleanup();
  return 0;
}
