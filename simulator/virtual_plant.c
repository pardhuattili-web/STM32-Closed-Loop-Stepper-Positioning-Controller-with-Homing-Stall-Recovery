#include "virtual_plant.h"
#include <stddef.h>
void virtual_plant_init(virtual_plant_t *p, int32_t initial, uint32_t delta) { if (p != NULL) { p->position_counts = initial; p->target_counts = initial; p->max_delta_per_tick = delta == 0U ? 1U : delta; p->stall_enabled = 0U; } }
void virtual_plant_set_target(virtual_plant_t *p, int32_t target) { if (p != NULL) p->target_counts = target; }
void virtual_plant_tick(virtual_plant_t *p) { int64_t delta, limit; if (p == NULL || p->stall_enabled != 0U) return; delta = (int64_t)p->target_counts - p->position_counts; limit = p->max_delta_per_tick; if (delta > limit) delta = limit; else if (delta < -limit) delta = -limit; p->position_counts += (int32_t)delta; }
