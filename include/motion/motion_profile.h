#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint32_t max_speed_counts_s; uint32_t accel_counts_s2; } motion_profile_config_t;
typedef struct { int32_t target_counts; int32_t start_counts; uint32_t max_speed_counts_s; uint32_t accel_counts_s2; } motion_profile_t;
bool motion_profile_init(motion_profile_t *profile, int32_t start_counts, int32_t target_counts, motion_profile_config_t config);
/* Returns a stopping-distance speed hint; hardware must still ramp pulse output. */
uint32_t motion_profile_speed_at(const motion_profile_t *profile, int32_t current_counts);
#endif
