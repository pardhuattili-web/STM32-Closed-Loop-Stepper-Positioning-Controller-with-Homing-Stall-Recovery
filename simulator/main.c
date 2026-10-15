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

    virtual_plant_init(&plant, 0, 40U);
    puts("Closed-loop stepper controller host demo (simulated)");

    if (!motion_controller_start_homing(&controller)) {
        fputs("could not enter homing state\n", stderr);
        return 1;
    }
    motion_controller_home_switch(&controller);
    printf("Homing complete: position=%" PRId32 " counts\n",
           controller.position_counts);

    if (!motion_controller_move_to(&controller, 1000)) {
        fputs("move command rejected\n", stderr);
        return 1;
    }
    virtual_plant_set_target(&plant, controller.target_counts);

    for (tick = 0U; tick < 30U; ++tick) {
        virtual_plant_tick(&plant);
        motion_controller_update_feedback(&controller, plant.position_counts);

        if (tick % 5U == 0U) {
            printf("tick=%u position=%" PRId32 " target=%" PRId32
                   " speed_hint=%" PRIu32 " state=%d\n",
                   tick, controller.position_counts, controller.target_counts,
                   motion_profile_speed_at(&profile, controller.position_counts),
                   (int)controller.state);
        }
        if (controller.state == MOTION_STATE_FAULT) {
            fprintf(stderr, "fault=%d\n", (int)controller.fault);
            return 2;
        }
    }

    puts("End of deterministic host demo; hardware remains unvalidated.");
    return 0;
}
