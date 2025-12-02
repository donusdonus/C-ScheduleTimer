
#ifndef __LIB__SCHEDULETIMER__H
#define __LIB__SCHEDULETIMER__H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define ___OS_WINDOWS___
// #define ___PLATFORM_ARUDINO___
#ifdef ___PLATFORM_ARDUINO___
#include <arduino.h>
#endif

#ifdef ___OS_WINDOWS___
#include <windows.h>
#include <chrono>
#include <iostream>
#endif

enum TimerMode
{
    ONESHORT = 0,
    PULSE = 1,
    CAPTURE = 2
};

#pragma pack(push, 1)
struct Timer
{
    uint32_t ton_ms; /* On time in milliseconds */
    uint32_t toff_ms; /* Off time in milliseconds */
    uint32_t toffset; /* Time offset in milliseconds */
    uint32_t accumulate; /* Accumulated time in milliseconds */
    union
    {
        struct
        {
            uint8_t done : 1; /* Timer done flag */
            uint8_t running : 1; /* Timer running flag */
            uint8_t reserved : 6;
        } bits;
        uint8_t raw; /* Status raw data */
    } status; /* Timer status */

    union
    {
        struct
        {
            uint8_t cmd_enable : 1; /* Command to enable timer */
            uint8_t cmd_mode : 4; /* Command timer mode */
            uint8_t reserved : 3; /* Reserved bits */
        } bits;
        uint8_t raw; /* Control raw data */
    } ctrl;
};
#pragma pack(pop)

#define TimerSize sizeof(Timer)
class ScheduleTimer
{
public:
    ScheduleTimer(); /* Constructor */
    ~ScheduleTimer(); /* Destructor */

    /* Config TimerMode::CAPTURE */
    void Config_capture_mode(Timer *tmr);

    /* Config TimerMode::ONESHOT */
    void Config_oneshot_mode(Timer *tmr, uint32_t Offtime_ms);

    /* Config TimerMode::PULSE */
    void Config_pulse_mode(Timer *tmr, uint32_t Ontime_msec, uint32_t Offtime_msec);

    /* Handle Timer All Type */
    Timer *Handle(Timer *tmr);

    /* update millis() time then scan all timer as handle */
    void Update();

    /* Reset Timer */
    void Reset(Timer *tmr);

    /* Enable or Disable Timer */
    void Enable(Timer *tmr, bool enable);

    /* Check Timer Done */
    bool IsDone (Timer *tmr);

    /* Check Timer Running */
    bool IsRunning (Timer *tmr);

    /* Get Elapsed Millisecond */
    uint32_t ElapsedMillisecond(Timer *tmr);

    /* Get Timer Mode */
    TimerMode GetMode(Timer *tmr);

#ifdef ___OS_WINDOWS___
    /* Get current millis() time */
    unsigned long millis();
#endif

private:
    unsigned long scantime = 0;

    /* Initialize Timer Structure */
    void init(Timer *tmr);
};

#endif // __LIB__SCHEDULETIMER__H
