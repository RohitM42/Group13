/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Implement the functions defined in base.h
 */

#include "base.h"

#include "tiny_math.h"

#include <webots/supervisor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>

#define SPEED 4.0
#define MAX_SPEED 0.5
#define SPEED_INCREMENT 0.05
#define DISTANCE_TOLERANCE 0.05
#define ANGLE_TOLERANCE 0.05

// robot geometry
#define WHEEL_RADIUS 0.05
#define LX 0.228  // longitudinal distance from robot's COM to wheel [m].
#define LY 0.158  // lateral distance from robot's COM to wheel [m].

typedef struct {
  Vector2 v_target;
  double alpha;
  bool reached;
} goto_struct;

static WbDeviceTag wheels[4];
static goto_struct goto_data;

static double robot_vx = 0.0;
static double robot_vy = 0.0;
static double robot_omega = 0.0;

// supervisor nodes to replace gps and compass 
static WbNodeRef self_node = NULL;
static WbFieldRef translation_field = NULL;
static WbFieldRef rotation_field = NULL;

static void base_set_wheel_velocity(WbDeviceTag t, double velocity) {
  wb_motor_set_position(t, INFINITY);
  wb_motor_set_velocity(t, velocity);
}

static void base_set_wheel_speeds_helper(const double speeds[4]) {
  int i;
  for (i = 0; i < 4; i++)
    base_set_wheel_velocity(wheels[i], speeds[i]);
}

void base_init() {
  int i;
  char wheel_name[16];
  for (i = 0; i < 4; i++) {
    sprintf(wheel_name, "wheel%d", (i + 1));
    wheels[i] = wb_robot_get_device(wheel_name);
  }
}

void base_reset() {
  static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);
  robot_vx = 0.0;
  robot_vy = 0.0;
  robot_omega = 0.0;
}

void base_forwards() {
  static double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_backwards() {
  static double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_left() {
  static double speeds[4] = {-SPEED, SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_right() {
  static double speeds[4] = {SPEED, -SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_left() {
  static double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_right() {
  static double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_move(double vx, double vy, double omega) {
  double speeds[4];
  speeds[0] = 1 / WHEEL_RADIUS * (vx + vy + (LX + LY) * omega);
  speeds[1] = 1 / WHEEL_RADIUS * (vx - vy - (LX + LY) * omega);
  speeds[2] = 1 / WHEEL_RADIUS * (vx - vy + (LX + LY) * omega);
  speeds[3] = 1 / WHEEL_RADIUS * (vx + vy - (LX + LY) * omega);
  base_set_wheel_speeds_helper(speeds);
  // printf("Speeds: vx=%.2f[m/s] vy=%.2f[m/s] ω=%.2f[rad/s]\n", vx, vy, omega);
}

void base_forwards_increment() {
  robot_vx += SPEED_INCREMENT;
  robot_vx = robot_vx > MAX_SPEED ? MAX_SPEED : robot_vx;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_backwards_increment() {
  robot_vx -= SPEED_INCREMENT;
  robot_vx = robot_vx < -MAX_SPEED ? -MAX_SPEED : robot_vx;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_turn_left_increment() {
  robot_omega += SPEED_INCREMENT;
  robot_omega = robot_omega > MAX_SPEED ? MAX_SPEED : robot_omega;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_turn_right_increment() {
  robot_omega -= SPEED_INCREMENT;
  robot_omega = robot_omega < -MAX_SPEED ? -MAX_SPEED : robot_omega;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_strafe_left_increment() {
  robot_vy += SPEED_INCREMENT;
  robot_vy = robot_vy > MAX_SPEED ? MAX_SPEED : robot_vy;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_strafe_right_increment() {
  robot_vy -= SPEED_INCREMENT;
  robot_vy = robot_vy < -MAX_SPEED ? -MAX_SPEED : robot_vy;
  base_move(robot_vx, robot_vy, robot_omega);
}

// new goto init
void base_goto_init() {
  self_node = wb_supervisor_node_get_self();
  if (!self_node) {
    fprintf(stderr, "base_goto_init: this controller is not a supervisor, goto disabled\n");
  } else {
    translation_field = wb_supervisor_node_get_field(self_node, "translation");
    rotation_field = wb_supervisor_node_get_field(self_node, "rotation");
    if (!translation_field || !rotation_field)
      fprintf(stderr, "base_goto_init: failed to get translation or rotation field\n");
  }

  goto_data.v_target.u = 0.0;
  goto_data.v_target.v = 0.0;
  goto_data.alpha = 0.0;
  goto_data.reached = true;
}

// new set target
void base_goto_set_target(double x, double y, double alpha) {
  goto_data.v_target.u = x;
  goto_data.v_target.v = y;
  goto_data.alpha = alpha;    // rad
  goto_data.reached = false;
}

// angle normalization helper (to remove possible errors)
static double normalize_angle(double angle) {
  return atan2(sin(angle), cos(angle));
}

// new run
void base_goto_run() {
  if (goto_data.reached)
    return;

  // If supervisor stuff is not available, we cannot run goto control
  if (!self_node || !translation_field || !rotation_field) {
    fprintf(stderr, "base_goto_run: supervisor fields not initialized, disabling goto\n");
    goto_data.reached = true;
    base_reset();
    return;
  }

  // get current robot pose(replaces gps/compass)
  const double *t = wb_supervisor_field_get_sf_vec3f(translation_field);
  const double *r = wb_supervisor_field_get_sf_rotation(rotation_field);

  const double x = t[0];
  const double y = t[1];

  double ax = r[0];
  double ay = r[1];
  double az = r[2];
  double angle = r[3];

  if (az < 0.0) {
    az = -az;
    angle = -angle;
  }
  double yaw = angle;

  const double dx = goto_data.v_target.u - x;
  const double dy = goto_data.v_target.v - y;
  const double dist = sqrt(dx * dx + dy * dy);

  double vx_r = 0.0;
  double vy_r = 0.0;
  double omega = 0.0;

  if (dist > DISTANCE_TOLERANCE) {
    double v_mag = dist;
    if (v_mag > MAX_SPEED)
      v_mag = MAX_SPEED;

    double vx_w = (dx / dist) * v_mag;
    double vy_w = (dy / dist) * v_mag;

    double c = cos(yaw);
    double s = sin(yaw);
    vx_r =  c * vx_w + s * vy_w;
    vy_r = -s * vx_w + c * vy_w;

    omega = 0.0;

    base_move(vx_r, vy_r, omega);
    goto_data.reached = false;
    return;
  }

  // Target orientation
  double alpha = goto_data.alpha;
  if (!isnan(alpha)) {
    double angle_error = normalize_angle(alpha - yaw);

    if (fabs(angle_error) > ANGLE_TOLERANCE) {
      double k_omega = 1.0;
      omega = k_omega * angle_error;

      if (omega >  MAX_SPEED) omega =  MAX_SPEED;
      if (omega < -MAX_SPEED) omega = -MAX_SPEED;

      vx_r = 0.0;
      vy_r = 0.0;

      base_move(vx_r, vy_r, omega);
      goto_data.reached = false;
      return;
    }
  }

  goto_data.reached = true;
  // stop the robot when reached
  base_move(0.0, 0.0, 0.0);
}


bool base_goto_reached() {
  return goto_data.reached;
}
