#include "ScheduleTimer.h"

int main() {
    Timer ledTimer;
    ScheduleTimer scheduler;

    // Configure: LED blinks with 500ms ON, 500ms OFF
    ledTimer.Config_pulse_mode(500, 500);
    ledTimer.Enable(true);

    while (1) {
        scheduler.Update();
    
        // Check if timer is currently in ON phase
        if (scheduler.Handle(&ledTimer)->IsDone()) 
        {
            printf("LED: ON\n");
        } else {
            printf("LED: OFF\n");
        }

    }
    
    return 0;
}