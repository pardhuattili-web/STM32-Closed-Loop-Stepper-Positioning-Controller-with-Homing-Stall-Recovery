#ifndef VIRTUAL_PLANT_H
#define VIRTUAL_PLANT_H
#include <stdint.h>
typedef struct { int32_t position_counts, target_counts; uint32_t max_delta_per_tick; uint8_t stall_enabled; } virtual_plant_t;
void virtual_plant_init(virtual_plant_t *plant, int32_t initial_position, uint32_t max_delta_per_tick);
void virtual_plant_set_target(virtual_plant_t *plant, int32_t target);
void virtual_plant_tick(virtual_plant_t *plant);
#endif
