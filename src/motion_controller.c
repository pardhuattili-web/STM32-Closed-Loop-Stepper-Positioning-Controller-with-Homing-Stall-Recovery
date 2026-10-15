#include "motion/motion_controller.h"

static void latch_fault(motion_controller_t *c, motion_fault_t fault)
{
    c->fault = fault;
    c->state = MOTION_STATE_FAULT;
    c->progress_clock_started = false;
}

bool motion_controller_init(motion_controller_t *c,
                            int32_t lo,
                            int32_t hi,
                            uint32_t error_limit)
{
    /* Homing defines coordinate zero, so zero must be inside soft limits. */
    if (c == 0 || lo >= hi || lo > 0 || hi < 0 || error_limit == 0U) {
        return false;
    }

    c->position_counts = 0;
    c->target_counts = 0;
    c->soft_min_counts = lo;
    c->soft_max_counts = hi;
    c->last_progress_counts = 0;
    c->following_error_limit_counts = error_limit;
    c->last_progress_ms = 0U;
    c->state = MOTION_STATE_UNHOMED;
    c->fault = MOTION_FAULT_NONE;
    c->home_switch_active = false;
    c->positive_limit_active = false;
    c->negative_limit_active = false;
    c->progress_clock_started = false;
    return true;
}

bool motion_controller_start_homing(motion_controller_t *c)
{
    if (c == 0 || c->state == MOTION_STATE_FAULT) {
        return false;
    }
    c->state = MOTION_STATE_HOMING;
    c->target_counts = c->position_counts;
    c->home_switch_active = false;
    c->progress_clock_started = false;
    return true;
}

void motion_controller_home_switch(motion_controller_t *c)
{
    if (c == 0) {
        return;
    }
    c->home_switch_active = true;
    if (c->state == MOTION_STATE_HOMING) {
        c->position_counts = 0;
        c->target_counts = 0;
        c->last_progress_counts = 0;
        c->state = MOTION_STATE_IDLE;
    }
}

bool motion_controller_move_to_at(motion_controller_t *c,
                                   int32_t target,
                                   uint32_t now_ms)
{
    if (c == 0 || c->state == MOTION_STATE_FAULT ||
        c->state == MOTION_STATE_UNHOMED) {
        return false;
    }
    if (target < c->soft_min_counts || target > c->soft_max_counts) {
        latch_fault(c, MOTION_FAULT_INVALID_COMMAND);
        return false;
    }

    if ((target > c->position_counts && c->positive_limit_active) ||
        (target < c->position_counts && c->negative_limit_active)) {
        latch_fault(c, MOTION_FAULT_LIMIT);
        return false;
    }

    c->target_counts = target;
    c->state = target == c->position_counts ? MOTION_STATE_IDLE : MOTION_STATE_MOVING;
    c->last_progress_counts = c->position_counts;
    c->last_progress_ms = now_ms;
    c->progress_clock_started = c->state == MOTION_STATE_MOVING;
    return true;
}

bool motion_controller_move_to(motion_controller_t *c, int32_t target)
{
    return motion_controller_move_to_at(c, target, 0U);
}

void motion_controller_update_feedback_at(motion_controller_t *c,
                                          int32_t measured,
                                          uint32_t now_ms)
{
    int64_t error;

    if (c == 0 || c->state == MOTION_STATE_FAULT) {
        return;
    }

    c->position_counts = measured;

    if (measured < c->soft_min_counts || measured > c->soft_max_counts) {
        latch_fault(c, MOTION_FAULT_LIMIT);
        return;
    }

    if (c->state == MOTION_STATE_MOVING &&
        measured != c->last_progress_counts) {
        c->last_progress_counts = measured;
        c->last_progress_ms = now_ms;
        c->progress_clock_started = true;
    }

    if (c->state == MOTION_STATE_MOVING && measured == c->target_counts) {
        c->state = MOTION_STATE_IDLE;
        c->progress_clock_started = false;
    }

    /* A final-position discrepancy is meaningful only after the motion
     * layer reports the move complete. During travel, use the no-progress timer. */
    if (c->state == MOTION_STATE_IDLE) {
        error = (int64_t)c->target_counts - measured;
        if (error < 0) {
            error = -error;
        }
        if ((uint64_t)error > c->following_error_limit_counts) {
            latch_fault(c, MOTION_FAULT_FOLLOWING_ERROR);
        }
    }
}

void motion_controller_update_feedback(motion_controller_t *c, int32_t measured)
{
    uint32_t timestamp = c == 0 ? 0U : c->last_progress_ms;
    motion_controller_update_feedback_at(c, measured, timestamp);
}

void motion_controller_check_stall(motion_controller_t *c,
                                   uint32_t now_ms,
                                   uint32_t timeout_ms)
{
    if (c == 0 || c->state != MOTION_STATE_MOVING ||
        !c->progress_clock_started || timeout_ms == 0U) {
        return;
    }

    if ((uint32_t)(now_ms - c->last_progress_ms) >= timeout_ms) {
        latch_fault(c, MOTION_FAULT_STALL);
    }
}

void motion_controller_set_limits(motion_controller_t *c,
                                  bool negative,
                                  bool positive)
{
    if (c == 0) {
        return;
    }

    c->negative_limit_active = negative;
    c->positive_limit_active = positive;

    if ((negative && c->state == MOTION_STATE_MOVING &&
         c->target_counts < c->position_counts) ||
        (positive && c->state == MOTION_STATE_MOVING &&
         c->target_counts > c->position_counts)) {
        latch_fault(c, MOTION_FAULT_LIMIT);
    }
}

void motion_controller_stop(motion_controller_t *c)
{
    if (c != 0 && c->state != MOTION_STATE_FAULT) {
        c->target_counts = c->position_counts;
        c->state = MOTION_STATE_IDLE;
        c->progress_clock_started = false;
    }
}

void motion_controller_clear_fault(motion_controller_t *c)
{
    if (c != 0 && c->state == MOTION_STATE_FAULT) {
        c->fault = MOTION_FAULT_NONE;
        c->state = MOTION_STATE_UNHOMED;
        c->target_counts = c->position_counts;
        c->progress_clock_started = false;
    }
}
