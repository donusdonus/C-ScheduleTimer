#include "ScheduleTimer.h"

int main() {
    Timer delayTimer;
    ScheduleTimer scheduler;

    printf("Starting alarm in 3 seconds...\n");
    
    // Trigger after 3000ms (3 seconds)
    scheduler.Config_oneshot_mode(&delayTimer, 3000);
    scheduler.Enable(&delayTimer, true);

    while (1) {
        scheduler.Update();
        scheduler.Handle(&delayTimer);

        if (scheduler.IsDone(&delayTimer)) {
            printf("BEEP! Alarm triggered!\n");
            break;
        }
        else 
        {
            // Show remaining time
            uint32_t elapsed = scheduler.ElapsedMillisecond(&delayTimer);
            printf("Elapsed: %u ms\n", elapsed);
        }
        
        
  
    }
    
    return 0;
}