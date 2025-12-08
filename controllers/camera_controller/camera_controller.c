/* QR Code decoding using quirc library
 * Source: https://github.com/dlbeer/quirc
 * Used for QR code detection in box sorting system
 */

#include <webots/robot.h>
#include <webots/camera.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "quirc/quirc.h"

#define TIME_STEP 32
#define COOLDOWN 10000 //10 seconds

int main() {
  wb_robot_init();

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  
  struct quirc *qr = quirc_new();
  if (!qr) {
    printf("Failed to create quirc object\n");
    return 1;
  }

  printf("Camera controller started with QR code detection.\n");

  int cooldown_timer = 0;
  bool code_detected = false;

  while (wb_robot_step(TIME_STEP) != -1) {

    if (cooldown_timer > 0) {
      cooldown_timer -= TIME_STEP;
      continue;
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
            printf("CHUTE A\n");
            cooldown_timer= COOLDOWN;
            code_detected = true;
            break;
          } else if (strcmp((char*)data.payload, "CHUTE_B") == 0) {
            printf("CHUTE B\n");
            cooldown_timer= COOLDOWN;
            code_detected = true;
            break;
          } else {
            printf("Unknown  %s\n", data.payload);
            cooldown_timer= COOLDOWN;
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
