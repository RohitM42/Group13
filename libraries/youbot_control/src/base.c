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
#define MAX_SPEED 0.3
#define SPEED_INCREMENT 0.05
#define DISTANCE_TOLERANCE 0.05
#define ANGLE_TOLERANCE 0.05

// robot geometry
#define WHEEL_RADIUS 0.05
#define LX 0.228  // longitudinal distance from robot's COM to wheel [m].
#define LY 0.158  // lateral distance from robot's COM to wheel [m].

// stimulus coefficients
#define K1 3.0
#define K2 1.0
#define K3 1.0

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
  printf("Speeds: vx=%.2f[m/s] vy=%.2f[m/s] ω=%.2f[rad/s]\n", vx, vy, omega);
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

// new run
void base_goto_run() {
  if (!self_node || !translation_field || !rotation_field) {
    fprintf(stderr, "base_goto_run: supervisor fields not available, goto disabled\n");
    goto_data.reached = true;
    return;
  }

  const double *pos = wb_supervisor_field_get_sf_vec3f(translation_field);
  const double *rot = wb_supervisor_field_get_sf_rotation(rotation_field);

  double x = pos[0];
  double y = pos[1];

  double yaw = rot[3];

  double dx = goto_data.v_target.u - x;
  double dy = goto_data.v_target.v - y;
  double distance = sqrt(dx * dx + dy * dy);

  double front_x = cos(yaw);
  double front_y = sin(yaw);

  double left_x  = -front_y;
  double left_y  =  front_x;

  double u  = front_x * dx + front_y * dy;   // forward error
  double ell = left_x  * dx + left_y  * dy;  // left error


  const double KP_POS = 0.8;  // position gain

  double vx = KP_POS * u;     // forward speed
  double vy = KP_POS * ell;   // strafe-left/right speed
  double omega = 0.0;         // no rotation for now

  // limit translational speed
  double v_norm = sqrt(vx * vx + vy * vy);
  if (v_norm > MAX_SPEED) {
    vx *= MAX_SPEED / v_norm;
    vy *= MAX_SPEED / v_norm;
  }

  // actually move the base
  base_move(vx, vy, omega);

  // dubug(remove later)

  printf(
    "[GOTO DEBUG]\n"
    "  Current pos: (%.3f, %.3f), yaw=%.3f rad\n"
    "  Target  pos: (%.3f, %.3f)\n"
    "  Errors: dist=%.4f\n"
    "  Robot-frame target: u=%.3f (forward), ell=%.3f (left)\n"
    "  Commanded v: vx=%.3f, vy=%.3f\n"
    "-------------------------------------------------------\n",
    x, y, yaw,
    goto_data.v_target.u, goto_data.v_target.v,
    distance,
    u, ell,
    vx, vy
  );

  // stop condition

  if (distance < DISTANCE_TOLERANCE)
    goto_data.reached = true;
}


bool base_goto_reached() {
  return goto_data.reached;
}
