#include <webots/robot.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>

int main() {
  wb_robot_init();

  int timestep = (int)wb_robot_get_basic_time_step();

  WbDeviceTag belt = wb_robot_get_device("belt_motor");
  if (belt == 0) {
    fprintf(stderr, "ERROR: belt_motor not found.\n");
    wb_robot_cleanup();
    return 1;
  }

  wb_motor_set_position(belt, INFINITY);
  wb_motor_set_velocity(belt, 0.2);  // m/s

  while (wb_robot_step(timestep) != -1) {
    // Conveyor belt running continuously
  }

  wb_robot_cleanup();
  return 0;
}

