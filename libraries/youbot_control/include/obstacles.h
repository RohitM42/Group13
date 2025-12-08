#ifndef OBSTACLES_H
#define OBSTACLES_H

typedef struct {
  double x;
  double y;
} ObstaclePoint;

typedef struct {
  char def_name[64];

  double center_x;
  double center_y;
  double size_x;
  double size_y;

  ObstaclePoint obstacle_points[4];  // inflated by robot radius
  ObstaclePoint nav_points[4];       // inflated by robot radius + clearance
} ObstacleInfo;

void obstacles_init(void);

int obstacles_register_rect_from_def(const char *def_name, double robot_radius);
void obstacles_register_rects_by_type(const char *type_name, double robot_radius);

int obstacles_get_count(void);

const ObstacleInfo *obstacles_get(int index);

#endif
