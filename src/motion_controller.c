#include "motion/motion_controller.h"
static void latch_fault(motion_controller_t *c, motion_fault_t f) { c->fault = f; c->state = MOTION_STATE_FAULT; }
bool motion_controller_init(motion_controller_t *c, int32_t lo, int32_t hi, uint32_t error_limit) {
    if (c == 0 || lo >= hi || error_limit == 0U) return false;
    c->position_counts = 0; c->target_counts = 0; c->soft_min_counts = lo; c->soft_max_counts = hi; c->following_error_limit_counts = error_limit;
    c->state = MOTION_STATE_UNHOMED; c->fault = MOTION_FAULT_NONE; c->home_switch_active = false; c->positive_limit_active = false; c->negative_limit_active = false; return true;
}
bool motion_controller_start_homing(motion_controller_t *c) { if (c == 0 || c->state == MOTION_STATE_FAULT) return false; c->state = MOTION_STATE_HOMING; c->target_counts = c->position_counts; return true; }
void motion_controller_home_switch(motion_controller_t *c) { if (c == 0) return; c->home_switch_active = true; if (c->state == MOTION_STATE_HOMING) { c->position_counts = 0; c->target_counts = 0; c->state = MOTION_STATE_IDLE; } }
bool motion_controller_move_to(motion_controller_t *c, int32_t target) {
    if (c == 0 || c->state == MOTION_STATE_FAULT || c->state == MOTION_STATE_UNHOMED) return false;
    if (target < c->soft_min_counts || target > c->soft_max_counts) { latch_fault(c, MOTION_FAULT_INVALID_COMMAND); return false; }
    c->target_counts = target; c->state = target == c->position_counts ? MOTION_STATE_IDLE : MOTION_STATE_MOVING; return true;
}
void motion_controller_update_feedback(motion_controller_t *c, int32_t measured) {
    int64_t error;
    if (c == 0 || c->state == MOTION_STATE_FAULT) return;
    c->position_counts = measured;
    if ((c->positive_limit_active && measured >= c->target_counts) || (c->negative_limit_active && measured <= c->target_counts)) { latch_fault(c, MOTION_FAULT_LIMIT); return; }
    error = (int64_t)c->target_counts - measured; if (error < 0) error = -error;
    if ((uint64_t)error > c->following_error_limit_counts) { latch_fault(c, MOTION_FAULT_FOLLOWING_ERROR); return; }
    if (c->state == MOTION_STATE_MOVING && measured == c->target_counts) c->state = MOTION_STATE_IDLE;
}
void motion_controller_set_limits(motion_controller_t *c, bool negative, bool positive) {
    if (c == 0) return;
    c->negative_limit_active = negative; c->positive_limit_active = positive;
    if ((negative && c->state == MOTION_STATE_MOVING && c->target_counts < c->position_counts) || (positive && c->state == MOTION_STATE_MOVING && c->target_counts > c->position_counts)) latch_fault(c, MOTION_FAULT_LIMIT);
}
void motion_controller_stop(motion_controller_t *c) { if (c != 0 && c->state != MOTION_STATE_FAULT) { c->target_counts = c->position_counts; c->state = MOTION_STATE_IDLE; } }
void motion_controller_clear_fault(motion_controller_t *c) { if (c != 0 && c->state == MOTION_STATE_FAULT) { c->fault = MOTION_FAULT_NONE; c->state = MOTION_STATE_UNHOMED; c->target_counts = c->position_counts; } }
