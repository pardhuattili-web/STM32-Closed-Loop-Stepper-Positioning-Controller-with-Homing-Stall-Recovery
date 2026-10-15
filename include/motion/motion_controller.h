#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { MOTION_STATE_UNHOMED = 0, MOTION_STATE_IDLE, MOTION_STATE_HOMING, MOTION_STATE_MOVING, MOTION_STATE_FAULT } motion_state_t;
typedef enum { MOTION_FAULT_NONE = 0, MOTION_FAULT_LIMIT, MOTION_FAULT_FOLLOWING_ERROR, MOTION_FAULT_INVALID_COMMAND } motion_fault_t;
typedef struct {
    int32_t position_counts, target_counts, soft_min_counts, soft_max_counts;
    uint32_t following_error_limit_counts;
    motion_state_t state;
    motion_fault_t fault;
    bool home_switch_active, positive_limit_active, negative_limit_active;
} motion_controller_t;
bool motion_controller_init(motion_controller_t *controller, int32_t soft_min_counts, int32_t soft_max_counts, uint32_t following_error_limit_counts);
bool motion_controller_start_homing(motion_controller_t *controller);
void motion_controller_home_switch(motion_controller_t *controller);
bool motion_controller_move_to(motion_controller_t *controller, int32_t target_counts);
/* Updates measured position. The baseline detects final position error when idle;
 * trajectory-aware stall monitoring requires expected-position/time-window inputs. */
void motion_controller_update_feedback(motion_controller_t *controller, int32_t measured_position_counts);
void motion_controller_set_limits(motion_controller_t *controller, bool negative_active, bool positive_active);
void motion_controller_stop(motion_controller_t *controller);
void motion_controller_clear_fault(motion_controller_t *controller);
#endif
