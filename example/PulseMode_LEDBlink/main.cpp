#include "ScheduleTimer.h"

int main() {
    Timer ledTimer;
    ScheduleTimer scheduler;

    // Configure: LED blinks with 500ms ON, 500ms OFF
    scheduler.Config_pulse_mode(&ledTimer, 500, 500);
    scheduler.Enable(&ledTimer, true);

    while (1) {
        scheduler.Update();
        scheduler.Handle(&ledTimer);

        // Check if timer is currently in ON phase
        if (scheduler.IsDone(&ledTimer)) {
            printf("LED: ON\n");
        } else {
            printf("LED: OFF\n");
        }

    }
    
    return 0;
}