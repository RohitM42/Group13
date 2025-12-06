#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdbool.h>

typedef enum {
  PALLET_1 = 0,
  PALLET_2 = 1,
} PalletId;

void nav_init(void);

void nav_start_from_conveyor_to(PalletId pallet);

void nav_update(void);

bool nav_is_active(void);

#endif
