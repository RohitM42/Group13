#include "gripper.h"

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/position_sensor.h>

#include "tiny_math.h"
#include <stdio.h>

#define LEFT 0
#define RIGHT 1

#define MIN_POS 0.0
#define MAX_POS 0.09999 //0.1 causes issues errors like 0.1 > 0.1
#define OFFSET_WHEN_LOCKED 0.021

static WbDeviceTag fingers;
static WbDeviceTag finger_sensor;  // position sensor

// Supervisor handle for the connector Transform scale (Robot field)
static WbFieldRef connector_scale_field = NULL;

// this scales the box between fingers for better visuals
static void update_connector_scale(double pos) {
  if (!connector_scale_field)
    return;

  // Map pos in [MIN_POS, MAX_POS] to t in [0, 1]
  double t = (pos - MIN_POS) / (MAX_POS - MIN_POS);
  if (t < 0.0)
    t = 0.0;
  else if (t > 1.0)
    t = 1.0;

  // Don't let the scale reach 0: keep it between 0.2 and 1.0
  const double min_scale_y = 0.25;   // closed gripper
  const double max_scale_y = 1.0;   // fully open
  double scale_y = min_scale_y + (max_scale_y - min_scale_y) * t;

  double scale[3] = {1.0, scale_y, 1.0};  // X, Y, Z

  // printf("pos=%.3f -> connectorScale [%.3f, %.3f, %.3f]\n", pos, scale[0], scale_y, scale[2]);

  wb_supervisor_field_set_sf_vec3f(connector_scale_field, scale);
}

void gripper_init() {
  printf(">>> CUSTOM GRIPPER INIT, MAX_POS = %f <<<\n", MAX_POS);

  fingers = wb_robot_get_device("finger::left");
  wb_motor_set_velocity(fingers, 0.03);

  finger_sensor = wb_robot_get_device("finger::leftsensor");
  if (finger_sensor) {
    int dt = (int)wb_robot_get_basic_time_step();
    wb_position_sensor_enable(finger_sensor, dt);
  } else {
    printf("WARNING: finger::leftsensor not found.\n");
  }

  WbNodeRef self = wb_supervisor_node_get_self();
  if (!self) {
    printf("WARNING: this controller is not running on a Supervisor robot.\n");
    return;
  }

  connector_scale_field = wb_supervisor_node_get_field(self, "connectorScale");
  if (!connector_scale_field) {
    printf("WARNING: connectorScale field not found on robot.\n");
    return;
  }

  double init_pos = MIN_POS;
  if (finger_sensor)
    init_pos = wb_position_sensor_get_value(finger_sensor);
  update_connector_scale(init_pos);
}

void gripper_grip() {
  wb_motor_set_position(fingers, MIN_POS);
}

void gripper_release() {
  wb_motor_set_position(fingers, MAX_POS);
}

void gripper_set_gap(double gap) {
  double v = bound(0.5 * (gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS);
  wb_motor_set_position(fingers, v);
}

void gripper_step(void) {
  if (!finger_sensor || !connector_scale_field)
    return;

  double pos = wb_position_sensor_get_value(finger_sensor);
  update_connector_scale(pos);
}
