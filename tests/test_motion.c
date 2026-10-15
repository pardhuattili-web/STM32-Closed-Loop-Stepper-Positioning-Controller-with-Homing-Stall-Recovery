#include "motion/motion_controller.h"
#include "motion/motion_profile.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

static void home_controller(motion_controller_t *c)
{
    CHECK(motion_controller_start_homing(c));
    motion_controller_home_switch(c);
    CHECK(c->state == MOTION_STATE_IDLE);
    CHECK(c->position_counts == 0);
}

static void test_init_validation(void)
{
    motion_controller_t c;
    CHECK(!motion_controller_init(&c, 100, -100, 10U));
    CHECK(!motion_controller_init(&c, -100, 100, 0U));
    CHECK(!motion_controller_init(&c, 1, 100, 10U));
    CHECK(motion_controller_init(&c, -1000, 1000, 25U));
    CHECK(c.state == MOTION_STATE_UNHOMED);
}

static void test_homing_and_move(void)
{
    motion_controller_t c;
    CHECK(motion_controller_init(&c, -1000, 1000, 20U));
    CHECK(!motion_controller_move_to(&c, 100));
    home_controller(&c);
    CHECK(motion_controller_move_to_at(&c, 200, 100U));
    motion_controller_update_feedback_at(&c, 100, 110U);
    CHECK(c.state == MOTION_STATE_MOVING);
    motion_controller_update_feedback_at(&c, 200, 120U);
    CHECK(c.state == MOTION_STATE_IDLE);
}

static void test_soft_limit_and_rehome(void)
{
    motion_controller_t c;
    CHECK(motion_controller_init(&c, -100, 100, 20U));
    home_controller(&c);
    CHECK(!motion_controller_move_to(&c, 101));
    CHECK(c.state == MOTION_STATE_FAULT);
    CHECK(c.fault == MOTION_FAULT_INVALID_COMMAND);
    motion_controller_clear_fault(&c);
    CHECK(c.state == MOTION_STATE_UNHOMED);
    CHECK(!motion_controller_move_to(&c, 10));
}

static void test_stall_detection(void)
{
    motion_controller_t c;
    CHECK(motion_controller_init(&c, -1000, 1000, 20U));
    home_controller(&c);
    CHECK(motion_controller_move_to_at(&c, 500, 1000U));
    motion_controller_update_feedback_at(&c, 0, 1020U);
    motion_controller_check_stall(&c, 1099U, 100U);
    CHECK(c.state == MOTION_STATE_MOVING);
    motion_controller_check_stall(&c, 1100U, 100U);
    CHECK(c.state == MOTION_STATE_FAULT);
    CHECK(c.fault == MOTION_FAULT_STALL);
}

static void test_progress_resets_stall_timer(void)
{
    motion_controller_t c;
    CHECK(motion_controller_init(&c, -1000, 1000, 20U));
    home_controller(&c);
    CHECK(motion_controller_move_to_at(&c, 100, 100U));
    motion_controller_update_feedback_at(&c, 10, 120U);
    motion_controller_check_stall(&c, 210U, 100U);
    CHECK(c.state == MOTION_STATE_MOVING);
    motion_controller_update_feedback_at(&c, 20, 220U);
    motion_controller_check_stall(&c, 319U, 100U);
    CHECK(c.state == MOTION_STATE_MOVING);
    motion_controller_check_stall(&c, 320U, 100U);
    CHECK(c.state == MOTION_STATE_FAULT);
    CHECK(c.fault == MOTION_FAULT_STALL);
}

static void test_limit_switch(void)
{
    motion_controller_t c;
    CHECK(motion_controller_init(&c, -1000, 1000, 20U));
    home_controller(&c);
    CHECK(motion_controller_move_to_at(&c, 500, 10U));
    motion_controller_set_limits(&c, false, true);
    CHECK(c.state == MOTION_STATE_FAULT);
    CHECK(c.fault == MOTION_FAULT_LIMIT);
}

static void test_idle_encoder_drift(void)
{
    motion_controller_t c;
    CHECK(motion_controller_init(&c, -1000, 1000, 20U));
    home_controller(&c);
    motion_controller_update_feedback_at(&c, 50, 100U);
    CHECK(c.state == MOTION_STATE_FAULT);
    CHECK(c.fault == MOTION_FAULT_FOLLOWING_ERROR);
}

static void test_stop_and_profile(void)
{
    motion_controller_t c;
    motion_profile_t p;
    motion_profile_config_t cfg = {1000U, 100U};
    CHECK(motion_controller_init(&c, -1000, 1000, 20U));
    home_controller(&c);
    CHECK(motion_controller_move_to_at(&c, 100, 0U));
    motion_controller_update_feedback_at(&c, 10, 10U);
    motion_controller_stop(&c);
    CHECK(c.state == MOTION_STATE_IDLE);
    CHECK(c.target_counts == 10);

    CHECK(motion_profile_init(&p, 0, 1000, cfg));
    CHECK(motion_profile_speed_at(&p, 1000) == 0U);
    CHECK(motion_profile_speed_at(&p, 0) <= 1000U);
    CHECK(!motion_profile_init(&p, 0, 1, (motion_profile_config_t){0U, 1U}));
    CHECK(motion_profile_speed_at(NULL, 0) == 0U);
}

int main(void)
{
    test_init_validation();
    test_homing_and_move();
    test_soft_limit_and_rehome();
    test_stall_detection();
    test_progress_resets_stall_timer();
    test_limit_switch();
    test_idle_encoder_drift();
    test_stop_and_profile();
    puts("All motion-controller tests passed.");
    return EXIT_SUCCESS;
}
