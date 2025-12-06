#include "navigation.h"

#include <stdio.h>
#include <stddef.h>
#include <math.h>

#include "base.h"

typedef struct {
  double x;
  double y;
  double yaw;
} Waypoint;

enum {
  WP_CONVEYOR = 0,
  WP_INTERSECTION_1,
  WP_INTERSECTION_2,
  WP_PALLET_1,
  WP_PALLET_2,
  NUM_WAYPOINTS
};

static const Waypoint waypoints[NUM_WAYPOINTS] = {
  //   x,      y,      yaw
  { -2.0,  -2.0,  0.0 },   // WP_CONVEYOR
  { -1.5,  -2.0,  0.0 },   // WP_INTERSECTION_1
  { -1.0,  -1.0,  0.0 },   // WP_INTERSECTION_2
  {  0.0,  -1.0,  1.57 },  // WP_PALLET_1
  {  0.5,  -1.0,  1.57 },  // WP_PALLET_2
};

// Paths

// From conveyor to pallet C -> I1 -> I2 -> P1
static const int path_conveyor_to_pallet1[] = {
  WP_CONVEYOR,
  WP_INTERSECTION_1,
  WP_INTERSECTION_2,
  WP_PALLET_1
};

// From conveyor to pallet 2: C -> I1 -> I2 -> P2
static const int path_conveyor_to_pallet2[] = {
  WP_CONVEYOR,
  WP_INTERSECTION_1,
  WP_INTERSECTION_2,
  WP_PALLET_2
};

static const int *current_path = NULL;
static int current_path_len = 0;
static int current_segment = 0;
static bool nav_active = false;

void nav_init(void) {
  current_path = NULL;
  current_path_len = 0;
  current_segment = 0;
  nav_active = false;
}

static void nav_start_path(const int *path, int path_len) {
  if (!path || path_len <= 0) {
    fprintf(stderr, "[NAV] nav_start_path called with invalid path\n");
    nav_active = false;
    return;
  }

  current_path = path;
  current_path_len = path_len;
  current_segment = 0;
  nav_active = true;

  int wp_index = current_path[current_segment];
  if (wp_index < 0 || wp_index >= NUM_WAYPOINTS) {
    fprintf(stderr, "[NAV] Invalid waypoint index %d\n", wp_index);
    nav_active = false;
    return;
  }

  Waypoint wp = waypoints[wp_index];
  base_goto_set_target(wp.x, wp.y, wp.yaw);
  printf("[NAV] Start path: first waypoint %d -> (%.3f, %.3f, %.3f)\n",
         wp_index, wp.x, wp.y, wp.yaw);
}

void nav_start_from_conveyor_to(PalletId pallet) {
  switch (pallet) {
    case PALLET_1:
      nav_start_path(path_conveyor_to_pallet1,
                     (int)(sizeof(path_conveyor_to_pallet1) / sizeof(int)));
      break;
    case PALLET_2:
      nav_start_path(path_conveyor_to_pallet2,
                     (int)(sizeof(path_conveyor_to_pallet2) / sizeof(int)));
      break;
    default:
      fprintf(stderr, "[NAV] Unknown pallet id %d\n", (int)pallet);
      nav_active = false;
      break;
  }
}

void nav_update(void) {
  if (!nav_active || !current_path)
    return;

  // If base_goto still working on current target, do nothing
  if (!base_goto_reached())
    return;

  // Reached current waypoint -> move to next
  current_segment++;
  if (current_segment >= current_path_len) {
    printf("[NAV] Path finished.\n");
    nav_active = false;
    base_reset();
    return;
  }

  int wp_index = current_path[current_segment];
  if (wp_index < 0 || wp_index >= NUM_WAYPOINTS) {
    fprintf(stderr, "[NAV] Invalid waypoint index %d\n", wp_index);
    nav_active = false;
    return;
  }

  Waypoint wp = waypoints[wp_index];
  printf("[NAV] Next waypoint %d -> (%.3f, %.3f, %.3f)\n",
         wp_index, wp.x, wp.y, wp.yaw);
  base_goto_set_target(wp.x, wp.y, wp.yaw);
}

bool nav_is_active(void) {
  return nav_active;
}
