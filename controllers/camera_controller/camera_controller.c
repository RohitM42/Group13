#include <webots/robot.h>
#include <webots/camera.h>
#include <stdio.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  printf("Camera controller started.\n");

  while (wb_robot_step(TIME_STEP) != -1) {
    const unsigned char *image = wb_camera_get_image(camera);
    if (image) {
      printf("cam ok");
    }
  }

  wb_robot_cleanup();
  return 0;
}
