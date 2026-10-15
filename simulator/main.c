#include "motion/motion_controller.h"
#include "motion/motion_profile.h"
#include "virtual_plant.h"
#include <inttypes.h>
#include <stdio.h>

int main(void)
{
    motion_controller_t controller;
    motion_profile_t profile;
    virtual_plant_t plant;
    const motion_profile_config_t config = {1200U, 600U};
    unsigned tick;

    if (!motion_controller_init(&controller, -5000, 5000, 80U) ||
        !motion_profile_init(&profile, 0, 1000, config)) {
        fputs("initialization failed\n", stderr);
        return 1;
    }

    virtual_plant_init(&plant, 0, 20U);
    puts("Closed-loop stepper controller host demo (simulated)");
    if (!motion_controller_start_homing(&controller)) {
        fputs("could not enter homing state\n", stderr);
        return 1;
    }
    motion_controller_home_switch(&controller);
    printf("Homing complete: position=%" PRId32 " counts\n", controller.position_counts);

    if (!motion_controller_move_to_at(&controller, 1000, 0U)) {
        fputs("move command rejected\n", stderr);
        return 1;
    }
    virtual_plant_set_target(&plant, controller.target_counts);

    for (tick = 1U; tick <= 50U; ++tick) {
        uint32_t now_ms = tick * 10U;
        virtual_plant_tick(&plant);
        motion_controller_update_feedback_at(&controller, plant.position_counts, now_ms);
        motion_controller_check_stall(&controller, now_ms, 100U);

        if (tick % 10U == 0U) {
            printf("t=%" PRIu32 " ms position=%" PRId32 " target=%" PRId32
                   " speed_hint=%" PRIu32 " state=%d\n",
                   now_ms, controller.position_counts, controller.target_counts,
                   motion_profile_speed_at(&profile, controller.position_counts),
                   (int)controller.state);
        }
        if (controller.state == MOTION_STATE_FAULT) {
            fprintf(stderr, "fault=%d\n", (int)controller.fault);
            return 2;
        }
    }

    if (controller.state != MOTION_STATE_IDLE || controller.position_counts != 1000) {
        fputs("simulated move did not reach its target\n", stderr);
        return 3;
    }

    /* Demonstrate bounded stall detection with a second commanded move. */
    if (!motion_controller_move_to_at(&controller, 1500, 500U)) {
        fputs("stall-test move rejected\n", stderr);
        return 4;
    }
    virtual_plant_set_target(&plant, 1500);
    plant.stall_enabled = 1U;
    motion_controller_check_stall(&controller, 599U, 100U);
    if (controller.state != MOTION_STATE_MOVING) {
        fputs("stall detected too early\n", stderr);
        return 5;
    }
    motion_controller_check_stall(&controller, 600U, 100U);
    if (controller.state != MOTION_STATE_FAULT || controller.fault != MOTION_FAULT_STALL) {
        fputs("stall was not detected\n", stderr);
        return 6;
    }
    puts("Injected no-motion fault detected at configured timeout.");
    puts("This is a deterministic host simulation; hardware remains unvalidated.");
    return 0;
}
