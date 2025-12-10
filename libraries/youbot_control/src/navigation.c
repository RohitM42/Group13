#include "navigation.h"

#include <stdio.h>
#include <stddef.h>
#include <math.h>
#include <float.h>
#include <stdbool.h>

#include <webots/supervisor.h>

#include "base.h"
#include "obstacles.h"

#define NAV_MAX_NODES      256
#define NAV_MAX_PATH_LEN   128

typedef struct {
  double x;
  double y;
} NavNode;

typedef struct {
  double x;
  double y;
  double yaw;
} Waypoint;

static bool nav_active = false;
static Waypoint path[NAV_MAX_PATH_LEN];
static int path_len = 0;
static int path_index = 0;

static WbNodeRef nav_self_node = NULL;
static WbFieldRef nav_translation_field = NULL;

static double distance2d(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

static void nav_get_robot_position(double *x, double *y) {
  if (!nav_translation_field) {
    if (!nav_self_node)
      nav_self_node = wb_supervisor_node_get_self();
    if (nav_self_node)
      nav_translation_field = wb_supervisor_node_get_field(nav_self_node, "translation");
  }

  if (!nav_translation_field) {
    fprintf(stderr, "[NAV] cannot get robot translation field\n");
    *x = 0.0;
    *y = 0.0;
    return;
  }

  const double *t = wb_supervisor_field_get_sf_vec3f(nav_translation_field);
  *x = t[0];
  *y = t[1];
}

static void rect_from_obstacle(const ObstacleInfo *o,
                               double *minx, double *maxx,
                               double *miny, double *maxy) {
  double mnx = o->obstacle_points[0].x;
  double mxx = o->obstacle_points[0].x;
  double mny = o->obstacle_points[0].y;
  double mxy = o->obstacle_points[0].y;
  for (int i = 1; i < 4; ++i) {
    double px = o->obstacle_points[i].x;
    double py = o->obstacle_points[i].y;
    if (px < mnx) mnx = px;
    if (px > mxx) mxx = px;
    if (py < mny) mny = py;
    if (py > mxy) mxy = py;
  }
  *minx = mnx;
  *maxx = mxx;
  *miny = mny;
  *maxy = mxy;
}

static bool point_in_rect_strict(double x, double y,
                                 double minx, double maxx,
                                 double miny, double maxy) {
  return (x > minx && x < maxx && y > miny && y < maxy);
}

typedef struct {
  double x;
  double y;
} Pt;

static int orientation(Pt a, Pt b, Pt c) {
  double val = (b.y - a.y) * (c.x - a.x) - (b.x - a.x) * (c.y - a.y);
  const double eps = 1e-9;
  if (fabs(val) < eps)
    return 0;
  return (val > 0.0) ? 1 : 2;
}

static bool on_segment(Pt a, Pt b, Pt c) {
  return (b.x <= fmax(a.x, c.x) + 1e-9 && b.x + 1e-9 >= fmin(a.x, c.x) &&
          b.y <= fmax(a.y, c.y) + 1e-9 && b.y + 1e-9 >= fmin(a.y, c.y));
}

static bool segments_intersect(Pt p1, Pt q1, Pt p2, Pt q2) {
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  if (o1 != o2 && o3 != o4)
    return true;

  if (o1 == 0 && on_segment(p1, p2, q1)) return true;
  if (o2 == 0 && on_segment(p1, q2, q1)) return true;
  if (o3 == 0 && on_segment(p2, p1, q2)) return true;
  if (o4 == 0 && on_segment(p2, q1, q2)) return true;

  return false;
}

static bool segment_intersects_any_obstacle(double x1, double y1,
                                            double x2, double y2) {
  Pt s = { x1, y1 };
  Pt e = { x2, y2 };

  int n_obs = obstacles_get_count();
  for (int i = 0; i < n_obs; ++i) {
    const ObstacleInfo *o = obstacles_get(i);
    if (!o)
      continue;

    double minx, maxx, miny, maxy;
    rect_from_obstacle(o, &minx, &maxx, &miny, &maxy);

    if (point_in_rect_strict(s.x, s.y, minx, maxx, miny, maxy))
      return true;
    if (point_in_rect_strict(e.x, e.y, minx, maxx, miny, maxy))
      return true;

    Pt p00 = { minx, miny };
    Pt p10 = { maxx, miny };
    Pt p11 = { maxx, maxy };
    Pt p01 = { minx, maxy };

    if (segments_intersect(s, e, p00, p10)) return true;
    if (segments_intersect(s, e, p10, p11)) return true;
    if (segments_intersect(s, e, p11, p01)) return true;
    if (segments_intersect(s, e, p01, p00)) return true;
  }
  return false;
}

void nav_init(void) {
  nav_active = false;
  path_len = 0;
  path_index = 0;
  nav_self_node = NULL;
  nav_translation_field = NULL;
}

void nav_start_to(double goal_x, double goal_y, double goal_yaw) {
  NavNode nodes[NAV_MAX_NODES];
  double graph[NAV_MAX_NODES][NAV_MAX_NODES];
  double g_score[NAV_MAX_NODES];
  double f_score[NAV_MAX_NODES];
  int came_from[NAV_MAX_NODES];
  bool open_set[NAV_MAX_NODES];
  bool closed_set[NAV_MAX_NODES];

  double start_x, start_y;
  nav_get_robot_position(&start_x, &start_y);

  int node_count = 0;

  nodes[node_count].x = start_x;
  nodes[node_count].y = start_y;
  int start_index = node_count;
  node_count++;

  nodes[node_count].x = goal_x;
  nodes[node_count].y = goal_y;
  int goal_index = node_count;
  node_count++;

  int n_obs = obstacles_get_count();
  for (int i = 0; i < n_obs; ++i) {
    const ObstacleInfo *o = obstacles_get(i);
    if (!o)
      continue;
    for (int j = 0; j < 4; ++j) {
      if (node_count >= NAV_MAX_NODES)
        break;
      nodes[node_count].x = o->nav_points[j].x;
      nodes[node_count].y = o->nav_points[j].y;
      node_count++;
    }
  }

  for (int i = 0; i < node_count; ++i) {
    for (int j = 0; j < node_count; ++j) {
      if (i == j) {
        graph[i][j] = 0.0;
      } else {
        double ax = nodes[i].x;
        double ay = nodes[i].y;
        double bx = nodes[j].x;
        double by = nodes[j].y;
        if (segment_intersects_any_obstacle(ax, ay, bx, by))
          graph[i][j] = DBL_MAX;
        else
          graph[i][j] = distance2d(ax, ay, bx, by);
      }
    }
  }

  for (int i = 0; i < node_count; ++i) {
    g_score[i] = DBL_MAX;
    f_score[i] = DBL_MAX;
    came_from[i] = -1;
    open_set[i] = false;
    closed_set[i] = false;
  }

  g_score[start_index] = 0.0;
  f_score[start_index] = distance2d(nodes[start_index].x, nodes[start_index].y,
                                    nodes[goal_index].x, nodes[goal_index].y);
  open_set[start_index] = true;

  bool found = false;

  while (1) {
    int current = -1;
    double best_f = DBL_MAX;
    for (int i = 0; i < node_count; ++i) {
      if (open_set[i] && f_score[i] < best_f) {
        best_f = f_score[i];
        current = i;
      }
    }

    if (current == -1)
      break;

    if (current == goal_index) {
      found = true;
      break;
    }

    open_set[current] = false;
    closed_set[current] = true;

    for (int nb = 0; nb < node_count; ++nb) {
      if (closed_set[nb])
        continue;
      if (graph[current][nb] == DBL_MAX)
        continue;

      double tentative_g = g_score[current] + graph[current][nb];
      if (tentative_g < g_score[nb]) {
        came_from[nb] = current;
        g_score[nb] = tentative_g;
        f_score[nb] = tentative_g +
                      distance2d(nodes[nb].x, nodes[nb].y,
                                 nodes[goal_index].x, nodes[goal_index].y);
        open_set[nb] = true;
      }
    }
  }

  if (!found) {
    fprintf(stderr, "[NAV] no path found to (%.3f, %.3f)\n", goal_x, goal_y);
    nav_active = false;
    path_len = 0;
    path_index = 0;
    return;
  }

  int recon[NAV_MAX_PATH_LEN];
  int recon_len = 0;
  int cur = goal_index;
  while (cur != -1 && recon_len < NAV_MAX_PATH_LEN) {
    recon[recon_len++] = cur;
    cur = came_from[cur];
  }

  if (recon_len == 0) {
    fprintf(stderr, "[NAV] reconstruction failed\n");
    nav_active = false;
    path_len = 0;
    path_index = 0;
    return;
  }

  path_len = 0;
  for (int i = recon_len - 1; i >= 0; --i) {
    if (path_len >= NAV_MAX_PATH_LEN)
      break;
    int idx = recon[i];
    path[path_len].x = nodes[idx].x;
    path[path_len].y = nodes[idx].y;
    path[path_len].yaw = NAN;
    path_len++;
  }

  path[path_len - 1].yaw = goal_yaw;

  path_index = 0;
  nav_active = true;

  base_goto_set_target(path[0].x, path[0].y, path[0].yaw);
}

void nav_update(void) {
  if (!nav_active)
    return;

  if (!base_goto_reached())
    return;

  path_index++;
  if (path_index >= path_len) {
    nav_active = false;
    base_reset();
    return;
  }

  Waypoint wp = path[path_index];
  base_goto_set_target(wp.x, wp.y, wp.yaw);
}

bool nav_is_active(void) {
  return nav_active;
}
