#include "obstacles.h"

#include <webots/supervisor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define MAX_OBSTACLES 64
#define NAV_CLEARANCE 0.10  // extra 10 cm outside the inflated obstacle

static ObstacleInfo g_obstacles[MAX_OBSTACLES];
static int g_obstacle_count = 0;
static bool g_supervisor_ok = false;

void obstacles_init(void) {
  WbNodeRef self = wb_supervisor_node_get_self();
  if (!self) {
    fprintf(stderr, "[OBST] controller is not a supervisor\n");
    g_supervisor_ok = false;
    return;
  }
  g_supervisor_ok = true;
  g_obstacle_count = 0;
}

static bool read_position_and_size(WbNodeRef node, double *cx, double *cy,
                                   double *sx, double *sy) {
  WbFieldRef t_field = wb_supervisor_node_get_field(node, "translation");
  if (!t_field) {
    fprintf(stderr, "[OBST] no translation field\n");
    return false;
  }
  const double *t = wb_supervisor_field_get_sf_vec3f(t_field);
  double x = t[0];
  double y = t[1];

  WbFieldRef size_field = wb_supervisor_node_get_field(node, "size");
  if (!size_field) {
    fprintf(stderr, "[OBST] no size field\n");
    return false;
  }
  const double *size = wb_supervisor_field_get_sf_vec3f(size_field);
  double sx_local = size[0];
  double sy_local = size[1];

  if (cx) *cx = x;
  if (cy) *cy = y;
  if (sx) *sx = sx_local;
  if (sy) *sy = sy_local;
  return true;
}

static double read_yaw(WbNodeRef node) {
  WbFieldRef r_field = wb_supervisor_node_get_field(node, "rotation");
  if (!r_field)
    return 0.0;

  const double *r = wb_supervisor_field_get_sf_rotation(r_field);
  double ax = r[0];
  double ay = r[1];
  double az = r[2];
  double angle = r[3];

  if (az < 0.0) {
    az = -az;
    angle = -angle;
  }
  return angle;
}

static void fill_obstacle_info(ObstacleInfo *o,
                               const char *name_or_type,
                               double cx, double cy,
                               double sx, double sy,
                               double yaw,
                               double robot_radius) {
  double half_x = sx * 0.5;
  double half_y = sy * 0.5;

  double Rx  = half_x + robot_radius;
  double Ry  = half_y + robot_radius;
  double RxN = Rx + NAV_CLEARANCE;
  double RyN = Ry + NAV_CLEARANCE;

  memset(o, 0, sizeof(*o));
  strncpy(o->def_name, name_or_type, sizeof(o->def_name) - 1);
  o->def_name[sizeof(o->def_name) - 1] = '\0';

  o->center_x = cx;
  o->center_y = cy;
  o->size_x   = sx;
  o->size_y   = sy;

  double c = cos(yaw);
  double s = sin(yaw);

  double dx_o[4] = { +Rx, +Rx, -Rx, -Rx };
  double dy_o[4] = { +Ry, -Ry, +Ry, -Ry };

  double dx_n[4] = { +RxN, +RxN, -RxN, -RxN };
  double dy_n[4] = { +RyN, -RyN, +RyN, -RyN };

  for (int i = 0; i < 4; ++i) {
    double wx = cx + c * dx_o[i] - s * dy_o[i];
    double wy = cy + s * dx_o[i] + c * dy_o[i];
    o->obstacle_points[i].x = wx;
    o->obstacle_points[i].y = wy;

    double wxn = cx + c * dx_n[i] - s * dy_n[i];
    double wyn = cy + s * dx_n[i] + c * dy_n[i];
    o->nav_points[i].x = wxn;
    o->nav_points[i].y = wyn;
  }
}

int obstacles_get_count(void) {
  return g_obstacle_count;
}

const ObstacleInfo *obstacles_get(int index) {
  if (index < 0 || index >= g_obstacle_count)
    return NULL;
  return &g_obstacles[index];
}

int obstacles_register_rect_from_def(const char *def_name, double robot_radius) {
  if (!g_supervisor_ok) {
    fprintf(stderr, "[OBST] not supervisor, cannot register '%s'\n", def_name);
    return -1;
  }

  if (g_obstacle_count >= MAX_OBSTACLES) {
    fprintf(stderr, "[OBST] max obstacles reached\n");
    return -1;
  }

  WbNodeRef node = wb_supervisor_node_get_from_def(def_name);
  if (!node) {
    fprintf(stderr, "[OBST] node '%s' not found\n", def_name);
    return -1;
  }

  double cx, cy, sx, sy;
  if (!read_position_and_size(node, &cx, &cy, &sx, &sy)) {
    fprintf(stderr, "[OBST] failed to read position/size for '%s'\n", def_name);
    return -1;
  }

  double yaw = read_yaw(node);

  ObstacleInfo *o = &g_obstacles[g_obstacle_count];
  fill_obstacle_info(o, def_name, cx, cy, sx, sy, yaw, robot_radius);

  int id = g_obstacle_count;
  g_obstacle_count++;

  printf("[OBST] %s: center=(%.3f,%.3f) size=(%.3f,%.3f) yaw=%.3f\n",
         def_name, cx, cy, sx, sy, yaw);
  printf("       obstacle rect points:\n");
  for (int i = 0; i < 4; ++i)
    printf("         o%d=(%.3f, %.3f)\n", i,
           o->obstacle_points[i].x, o->obstacle_points[i].y);
  printf("       nav rect points:\n");
  for (int i = 0; i < 4; ++i)
    printf("         n%d=(%.3f, %.3f)\n", i,
           o->nav_points[i].x, o->nav_points[i].y);

  return id;
}

static int register_rect_for_node(WbNodeRef node,
                                  const char *name_or_type,
                                  double robot_radius) {
  if (g_obstacle_count >= MAX_OBSTACLES) {
    fprintf(stderr, "[OBST] max obstacles reached\n");
    return -1;
  }

  double cx, cy, sx, sy;
  if (!read_position_and_size(node, &cx, &cy, &sx, &sy)) {
    fprintf(stderr, "[OBST] failed to read position/size for '%s'\n", name_or_type);
    return -1;
  }

  double yaw = read_yaw(node);

  ObstacleInfo *o = &g_obstacles[g_obstacle_count];
  fill_obstacle_info(o, name_or_type, cx, cy, sx, sy, yaw, robot_radius);

  int id = g_obstacle_count;
  g_obstacle_count++;

  printf("[OBST] %s (auto): center=(%.3f,%.3f) size=(%.3f,%.3f) yaw=%.3f\n",
         name_or_type, cx, cy, sx, sy, yaw);
  return id;
}

static void register_in_subtree_by_type(WbNodeRef node,
                                        const char *type_name,
                                        double robot_radius) {
  if (!node)
    return;

  const char *tname = wb_supervisor_node_get_type_name(node);
  if (tname && strcmp(tname, type_name) == 0) {
    const char *def = wb_supervisor_node_get_def(node);
    const char *label = def && def[0] ? def : tname;
    register_rect_for_node(node, label, robot_radius);
  }

  WbFieldRef children_field = wb_supervisor_node_get_field(node, "children");
  if (!children_field)
    return;

  int count = wb_supervisor_field_get_count(children_field);
  for (int i = 0; i < count; ++i) {
    WbNodeRef child = wb_supervisor_field_get_mf_node(children_field, i);
    register_in_subtree_by_type(child, type_name, robot_radius);
  }
}

void obstacles_register_rects_by_type(const char *type_name, double robot_radius) {
  if (!g_supervisor_ok) {
    fprintf(stderr, "[OBST] not supervisor, cannot auto-register type '%s'\n", type_name);
    return;
  }

  WbNodeRef root = wb_supervisor_node_get_root();
  register_in_subtree_by_type(root, type_name, robot_radius);
}