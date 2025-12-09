#include "ScheduleTimer.h"

int main()
{
    Timer lightTimer;
    ScheduleTimer scheduler;
    int state = -1; //-1=Prepare 0=Red, 1=Green, 2=Yellow

    lightTimer.Config_oneshot_mode(1000);
    lightTimer.Enable(true);

    while (1)
    {
        scheduler.Update();

        if (scheduler.Handle(&lightTimer)->IsDone())
        {

            state = (state + 1) % 3;

            switch (state)
            {
            case 0:
                printf("[RED] - Stop (5 seconds)\n");
                lightTimer.Config_oneshot_mode(5000);
               
                break;
            case 1:
                printf("[GREEN] - Go (3 seconds)\n");
                lightTimer.Config_oneshot_mode(3000);
            
                break;
            case 2:
                printf("[YELLOW] - Caution (1 second)\n");
                lightTimer.Config_oneshot_mode(1000);
           
                break;
            }

            lightTimer.Enable(true);

        }
    }

    return 0;
}