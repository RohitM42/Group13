#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdbool.h>

void nav_init(void);
void nav_start_to(double goal_x, double goal_y, double goal_yaw);
void nav_update(void);
bool nav_is_active(void);

#endif
