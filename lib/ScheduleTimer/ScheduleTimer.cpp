#include "ScheduleTimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ScheduleTimer::ScheduleTimer()
{
    
}

ScheduleTimer::~ScheduleTimer()
{

}


Timer * ScheduleTimer::Handle(Timer *tmr)
{
    /* check enable */
    if(tmr->ctrl.bits.cmd_enable == 0)
    {
        tmr->status.bits.running = 0;      
        tmr->status.bits.done = 0;  
        return tmr;
    }
    
    /* interlock running */
    if(tmr->status.bits.running == 0)
    {
        tmr->status.bits.running = 1;
        tmr->toffset = scantime;
    }

    switch(tmr->ctrl.bits.cmd_mode)
    {
        case TimerMode::ONESHORT:
            
            if(tmr->status.bits.done == 0)
            {
                tmr->accumulate = scantime - tmr->toffset;
                if(tmr->accumulate >= tmr->toff_ms)
                {
                    tmr->status.bits.done = 1;
                }
            }

            break;

        case TimerMode::PULSE:
            
            tmr->accumulate = scantime - tmr->toffset;

            if((tmr->status.bits.done == 0) && (tmr->accumulate >= tmr->toff_ms))
            {
                    tmr->status.bits.done = 1;
                    tmr->toffset = scantime;
                    tmr->accumulate = 0;
            }
            else if((tmr->status.bits.done == 1) && (tmr->accumulate >= tmr->ton_ms)) 
            {
                    tmr->status.bits.done = 0;
                    tmr->toffset = scantime;
                    tmr->accumulate = 0;

            }
            
            break;

        case TimerMode::CAPTURE:
            //Capture mode not implement yet.
            tmr->accumulate = scantime - tmr->toffset;
            break;

        default:
            break;
    }

    return tmr;
}


#ifdef ___OS_WINDOWS___
unsigned long ScheduleTimer::millis()
{
        using namespace std::chrono;
        return (unsigned long)duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()
        ).count();
}
#endif