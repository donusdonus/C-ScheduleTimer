#include "ScheduleTimer.h"

int main() {
    Timer delayTimer;
    ScheduleTimer scheduler;

    printf("Starting alarm in 3 seconds...\n");
    
    // Trigger after 3000ms (3 seconds)
    delayTimer.Config_oneshot_mode(3000);
    delayTimer.Enable(true);

    while (1) {
        scheduler.Update();
        

        if (scheduler.Handle(&delayTimer)->IsDone()) {
            printf("BEEP! Alarm triggered!\n");
            break;
        }
        else 
        {
            // Show remaining time
            printf("Elapsed: %u ms\n", delayTimer.ElapsedMillisecond());
        }
        
        
  
    }
    
    return 0;
}