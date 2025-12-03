#include "ScheduleTimer.h"

int main()
{
    Timer lightTimer;
    ScheduleTimer scheduler;
    int state = -1; //-1=Prepare 0=Red, 1=Green, 2=Yellow

    scheduler.Config_oneshot_mode(&lightTimer, 1000);
    scheduler.Enable(&lightTimer, true);

    while (1)
    {
        scheduler.Update();
        scheduler.Handle(&lightTimer);

        if (scheduler.IsDone(&lightTimer))
        {

            state = (state + 1) % 3;

            switch (state)
            {
            case 0:
                printf("[RED] - Stop (5 seconds)\n");
                scheduler.Config_oneshot_mode(&lightTimer, 5000);
               
                break;
            case 1:
                printf("[GREEN] - Go (3 seconds)\n");
                scheduler.Config_oneshot_mode(&lightTimer, 3000);
            
                break;
            case 2:
                printf("[YELLOW] - Caution (1 second)\n");
                scheduler.Config_oneshot_mode(&lightTimer, 1000);
           
                break;
            }

             scheduler.Enable(&lightTimer, true);

        }
    }

    return 0;
}