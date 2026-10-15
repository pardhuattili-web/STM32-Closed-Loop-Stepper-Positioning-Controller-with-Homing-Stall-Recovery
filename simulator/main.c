#include "motion/motion_controller.h"
#include "motion/motion_profile.h"
#include "virtual_plant.h"
#include <inttypes.h>
#include <stdio.h>
int main(void) {
    motion_controller_t c; motion_profile_t p; virtual_plant_t plant; motion_profile_config_t cfg = {1200U, 600U}; unsigned tick;
    if (!motion_controller_init(&c, -5000, 5000, 80U) || !motion_profile_init(&p, 0, 1000, cfg)) { fputs("init failed\n", stderr); return 1; }
    virtual_plant_init(&plant, 0, 40U); puts("Closed-loop stepper controller host demo (simulated)");
    if (!motion_controller_start_homing(&c)) return 1;
    motion_controller_home_switch(&c); printf("Homing complete: position=%" PRId32 " counts\n", c.position_counts);
    if (!motion_controller_move_to(&c, 1000)) return 1;
    virtual_plant_set_target(&plant, c.target_counts);
    for (tick = 0U; tick < 30U; ++tick) {
        virtual_plant_tick(&plant); motion_controller_update_feedback(&c, plant.position_counts);
        if (tick % 5U == 0U) printf("tick=%u position=%" PRId32 " target=%" PRId32 " speed_hint=%" PRIu32 " state=%d\n", tick, c.position_counts, c.target_counts, motion_profile_speed_at(&p, c.position_counts), (int)c.state);
        if (c.state == MOTION_STATE_FAULT) { fprintf(stderr, "fault=%d\n", (int)c.fault); return 2; }
    }
    puts("End of deterministic host demo; hardware remains unvalidated."); return 0;
}
