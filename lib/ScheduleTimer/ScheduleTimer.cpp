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

/* Config TimerMode::CAPTURE */
void ScheduleTimer::Config_capture_mode(Timer *tmr)
{
    init(tmr);
    tmr->ctrl.bits.cmd_mode = TimerMode::CAPTURE;
}

/* Config TimerMode::ONESHOT */
void ScheduleTimer::Config_oneshot_mode(Timer *tmr, uint32_t Offtime_ms)
{
    init(tmr);
    tmr->ctrl.bits.cmd_mode = TimerMode::ONESHORT;
    tmr->toff_ms = Offtime_ms;
    tmr->status.bits.running = 0;
}

/* Config TimerMode::PULSE */
void ScheduleTimer::Config_pulse_mode(Timer *tmr, uint32_t Ontime_msec, uint32_t Offtime_msec)
{
    init(tmr);
    tmr->ctrl.bits.cmd_mode = TimerMode::PULSE;
    tmr->ton_ms = Ontime_msec;
    tmr->toff_ms = Offtime_msec;
}

void ScheduleTimer::init(Timer *tmr)
{
    memset(tmr,0x00,TimerSize);
}

void ScheduleTimer::Update()
{
    scantime = millis();
}

void ScheduleTimer::Reset(Timer *tmr)
{
    tmr->status.bits.done = 0;
    tmr->status.bits.running = 0;
    tmr->accumulate = 0;
    tmr->toffset = 0;
}   

void ScheduleTimer::Enable(Timer *tmr,bool enable)
{
    tmr->ctrl.bits.cmd_enable = enable ? 1 : 0;
}

bool ScheduleTimer::IsDone (Timer *tmr)
{
    return tmr->status.bits.done ? true : false;
}

bool ScheduleTimer::IsRunning (Timer *tmr)
{
    return tmr->status.bits.running ? true : false;
}

uint32_t ScheduleTimer::ElapsedMillisecond(Timer *tmr)
{
    return tmr->accumulate;
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

            if(tmr->status.bits.done == 0)
            {
                if(tmr->accumulate >= tmr->toff_ms)
                {
                    tmr->status.bits.done = 1;
                    tmr->toffset = scantime;
                    tmr->accumulate = 0;
                }
            }
            else if(tmr->status.bits.done == 1) 
            {
                if(tmr->accumulate >= tmr->ton_ms)
                {
                    tmr->status.bits.done = 0;
                    tmr->toffset = scantime;
                    tmr->accumulate = 0;
                }
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

TimerMode ScheduleTimer::GetMode(Timer *tmr)
{
    return (TimerMode)(tmr->ctrl.bits.cmd_mode);
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