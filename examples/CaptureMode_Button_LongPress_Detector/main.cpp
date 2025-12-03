#include "ScheduleTimer.h"

int main() {
    Timer buttonTimer;
    ScheduleTimer scheduler;

    printf("Press button (Enter to simulate press/release)...\n");

    // Start measuring time
    buttonTimer.Config_capture_mode();
    buttonTimer.Enable(true);

    printf("Button pressed...\n");

    while(true)
    {
        scheduler.Update();
        scheduler.Handle(&buttonTimer);

        // Read total time
        uint32_t pressTime = buttonTimer.ElapsedMillisecond();
        printf("Button was pressed for: %u ms\n", pressTime);

        if (pressTime > 2000) {
            printf("Long press detected!\n");
            return 0;
        } else {
            printf("Short press detected\n");
        }
    }

    return 0;
}