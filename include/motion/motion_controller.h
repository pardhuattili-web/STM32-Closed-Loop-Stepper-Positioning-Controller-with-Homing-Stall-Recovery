#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_STATE_UNHOMED = 0,
    MOTION_STATE_IDLE,
    MOTION_STATE_HOMING,
    MOTION_STATE_MOVING,
    MOTION_STATE_FAULT
} motion_state_t;

typedef enum {
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_LIMIT,
    MOTION_FAULT_FOLLOWING_ERROR,
    MOTION_FAULT_INVALID_COMMAND,
    MOTION_FAULT_STALL
} motion_fault_t;

typedef struct {
    int32_t position_counts;
    int32_t target_counts;
    int32_t soft_min_counts;
    int32_t soft_max_counts;
    int32_t last_progress_counts;
    uint32_t following_error_limit_counts;
    uint32_t last_progress_ms;
    motion_state_t state;
    motion_fault_t fault;
    bool home_switch_active;
    bool positive_limit_active;
    bool negative_limit_active;
    bool progress_clock_started;
} motion_controller_t;

bool motion_controller_init(motion_controller_t *controller,
                            int32_t soft_min_counts,
                            int32_t soft_max_counts,
                            uint32_t following_error_limit_counts);
bool motion_controller_start_homing(motion_controller_t *controller);
void motion_controller_home_switch(motion_controller_t *controller);
bool motion_controller_move_to(motion_controller_t *controller, int32_t target_counts);
bool motion_controller_move_to_at(motion_controller_t *controller,
                                   int32_t target_counts,
                                   uint32_t now_ms);
void motion_controller_update_feedback(motion_controller_t *controller,
                                       int32_t measured_position_counts);
void motion_controller_update_feedback_at(motion_controller_t *controller,
                                          int32_t measured_position_counts,
                                          uint32_t now_ms);
/* Latches STALL if the controller is moving and measured position has not
 * changed for timeout_ms. Use a monotonic millisecond tick; unsigned subtraction
 * is rollover-safe for intervals shorter than half the uint32_t range. */
void motion_controller_check_stall(motion_controller_t *controller,
                                   uint32_t now_ms,
                                   uint32_t timeout_ms);
void motion_controller_set_limits(motion_controller_t *controller,
                                  bool negative_active,
                                  bool positive_active);
void motion_controller_stop(motion_controller_t *controller);
void motion_controller_clear_fault(motion_controller_t *controller);

#endif
