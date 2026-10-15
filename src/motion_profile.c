#include "motion/motion_profile.h"
bool motion_profile_init(motion_profile_t *profile, int32_t start_counts, int32_t target_counts, motion_profile_config_t config) {
    if (profile == 0 || config.max_speed_counts_s == 0U || config.accel_counts_s2 == 0U) return false;
    profile->start_counts = start_counts; profile->target_counts = target_counts;
    profile->max_speed_counts_s = config.max_speed_counts_s; profile->accel_counts_s2 = config.accel_counts_s2;
    return true;
}
static uint64_t isqrt_u64(uint64_t n) {
    uint64_t result = 0U, bit = 1ULL << 62;
    while (bit > n) bit >>= 2;
    while (bit != 0U) { if (n >= result + bit) { n -= result + bit; result = (result >> 1) + bit; } else result >>= 1; bit >>= 2; }
    return result;
}
uint32_t motion_profile_speed_at(const motion_profile_t *profile, int32_t current_counts) {
    int64_t remaining; uint64_t distance, speed_squared, speed;
    if (profile == 0) return 0U;
    remaining = (int64_t)profile->target_counts - (int64_t)current_counts;
    if (remaining == 0) return 0U;
    distance = (uint64_t)(remaining < 0 ? -remaining : remaining);
    if (distance > (uint64_t)INT32_MAX) distance = (uint64_t)INT32_MAX;
    speed_squared = 2ULL * (uint64_t)profile->accel_counts_s2 * distance;
    speed = isqrt_u64(speed_squared);
    if (speed > profile->max_speed_counts_s) speed = profile->max_speed_counts_s;
    return (uint32_t)speed;
}
