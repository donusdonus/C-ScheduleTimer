# ⏱️ ScheduleTimer - Software Timer Library

A lightweight, memory-efficient software timer library designed for embedded systems (Arduino, PLC, Windows) and real-time applications.

---

## ✨ Why Use ScheduleTimer?

- 🎯 **Ultra Small** - Only **18 bytes** per timer instance
- 🔒 **No Dynamic Memory** - Safe for real-time systems, no malloc/free
- ⚡ **Easy to Use** - Get started in 3 lines of code
- 🔄 **3 Timer Modes** - ONE-SHOT, PULSE, CAPTURE
- 🌍 **Cross-Platform** - Works on Arduino, Windows, any MCU

---

## 📚 Timer Modes Explained

### 🎯 Mode 1: ONE-SHOT (One-Time Delay)
Trigger an action after a fixed delay, then stop automatically.

```
Enable → Count Down → Done → Stop
```

**Real-world example:** Beep sound after 5 seconds

---

### 🔄 Mode 2: PULSE (Repeating Cycle)
Toggle output repeatedly with configurable ON/OFF times (like PWM).

```
ON (ton) → OFF (toff) → ON (ton) → OFF (toff) → ...
```

**Real-world example:** LED blinks every 2 seconds (1s on, 1s off)

---

### ⏳ Mode 3: CAPTURE (Time Accumulation)
Measure elapsed time from start to stop.

```
Enable → Accumulate Time → Disable → Read Total
```

**Real-world example:** Measure how long a button was pressed

---

## 🚀 Quick Start

### Step 1: Create Timer Instance
```cpp
#include "ScheduleTimer.h"

Timer myTimer;                    // Create timer structure
ScheduleTimer scheduler;          // Create scheduler
```

### Step 2: Configure Timer Mode
```cpp
// ONE-SHOT: Trigger after 2000ms
scheduler.Config_oneshot_mode(&myTimer, 2000);

// PULSE: Toggle every 1s on, 1s off
scheduler.Config_pulse_mode(&myTimer, 1000, 1000);

// CAPTURE: Start measuring time
scheduler.Config_capture_mode(&myTimer);
```

### Step 3: Enable & Update
```cpp
// Enable the timer
scheduler.Enable(&myTimer, true);

// Call this in your main loop (every 1-10ms)
scheduler.Update();

// Check timer status
if (scheduler.IsDone(&myTimer)) {
    printf("Timer finished!\n");
// ปิดตัวจับเวลาเพื่อหยุดการทำงานซ้ำ
scheduler.Enable(&myTimer, false);

// อ่านและแสดงเวลาที่ผ่านไป
uint32_t elapsed = scheduler.ElapsedMillisecond(&myTimer);
printf("Elapsed: %u ms\n", elapsed);

// รีเซ็ตถ้าต้องการใช้ซ้ำ
scheduler.Reset(&myTimer);

// ตัวอย่าง: เริ่มใหม่เป็น ONE-SHOT อีกครั้ง (2000 ms)
scheduler.Config_oneshot_mode(&myTimer, 2000);
scheduler.Enable(&myTimer, true);
```

---

## 📖 Complete Examples

### Example 1: LED Blink (PULSE Mode)
```cpp
#include "ScheduleTimer.h"

int main() {
    Timer ledTimer;
    ScheduleTimer scheduler;

    // Configure: LED blinks with 500ms ON, 500ms OFF
    scheduler.Config_pulse_mode(&ledTimer, 500, 500);
    scheduler.Enable(&ledTimer, true);

    while (1) {
        scheduler.Update();

        // Check if timer is currently in ON phase
        if (scheduler.IsRunning(&ledTimer)) {
            printf("LED: ON\n");
        } else {
            printf("LED: OFF\n");
        }

        Sleep(100);  // Small delay
    }
    
    return 0;
}
```

---

### Example 2: Delayed Action (ONE-SHOT Mode)
```cpp
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
```

---

### Example 3: Button Long-Press Detector (CAPTURE Mode)
```cpp
#include "ScheduleTimer.h"

int main() {
    Timer buttonTimer;
    ScheduleTimer scheduler;

    printf("Press button (Enter to simulate press/release)...\n");

    // Start measuring time
    scheduler.Config_capture_mode(&buttonTimer);
    scheduler.Enable(&buttonTimer, true);

    printf("Button pressed...\n");

    while(true)
    {
        scheduler.Update();
        scheduler.Handle(&buttonTimer);

        // Read total time
        uint32_t pressTime = scheduler.ElapsedMillisecond(&buttonTimer);
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
```

---

### Example 4: Traffic Light Cycle
```cpp
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
```

---

### Example 5: Periodic Sensor Reading
```cpp
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
```

---

## 🔧 API Reference

### Configuration Methods
| Method | Parameters | Purpose |
|--------|-----------|---------|
| `Config_oneshot_mode()` | `Timer *tmr, uint32_t Offtime_ms` | Set one-time delay |
| `Config_pulse_mode()` | `Timer *tmr, uint32_t Ontime_msec, uint32_t Offtime_msec` | Set repeating ON/OFF |
| `Config_capture_mode()` | `Timer *tmr` | Start time measurement |

### Control Methods
| Method | Parameters | Purpose |
|--------|-----------|---------|
| `Enable()` | `Timer *tmr, bool enable` | Start/Stop timer |
| `Reset()` | `Timer *tmr` | Reset timer to zero |
| `Update()` | (none) | Process all timers |
| `Handle()` | `Timer *tmr` | Manually handle one timer |

### Status Methods
| Method | Returns | Purpose |
|--------|---------|---------|
| `IsDone()` | `bool` | Is timer finished? |
| `IsRunning()` | `bool` | Is timer currently ON? |
| `ElapsedMillisecond()` | `uint32_t` | Get elapsed time in ms |
| `GetMode()` | `TimerMode` | Get current timer mode |

---

## 💡 Common Patterns

### Pattern 1: Timeout Check
```cpp
scheduler.Config_oneshot_mode(&timeoutTimer, 5000);
scheduler.Enable(&timeoutTimer, true);

while (!dataReceived) {
    scheduler.Update();
    if (scheduler.IsDone(&timeoutTimer)) {
        printf("Timeout! No data received.\n");
        break;
    }
}
```

### Pattern 2: Multi-State Machine
```cpp
enum State { IDLE, WAITING, ACTIVE, DONE };
State currentState = IDLE;

scheduler.Update();

switch (currentState) {
    case IDLE:
        scheduler.Config_oneshot_mode(&stateTimer, 1000);
        scheduler.Enable(&stateTimer, true);
        currentState = WAITING;
        break;
    
    case WAITING:
        if (scheduler.IsDone(&stateTimer)) {
            currentState = ACTIVE;
        }
        break;
}
```

### Pattern 3: Frequency Control
```cpp
// Run function exactly 100 times per second
scheduler.Config_pulse_mode(&freqTimer, 1, 9);  // 10ms cycle
scheduler.Enable(&freqTimer, true);

scheduler.Update();
if (scheduler.IsRunning(&freqTimer)) {
    myFunction();  // Called 100 times/sec
}
```

---

## ⚙️ Platform Support

| Platform | Status | Setup |
|----------|--------|-------|
| **Arduino** | ✅ Supported | Uncomment `#define ___PLATFORM_ARDUINO___` |
| **Windows** | ✅ Supported | Define `___OS_WINDOWS___` (default) |
| **Linux** | ✅ Supported | Requires `<chrono>` |
| **Generic MCU** | ✅ Supported | Customize `millis()` function |

---

## 📐 Memory Footprint

```
Per Timer Instance:
├── ton_ms:        4 bytes
├── toff_ms:       4 bytes
├── toffset:       4 bytes
├── accumulate:    4 bytes
└── status/ctrl:   2 bytes
└─────────────────
   Total:         18 bytes

ScheduleTimer Class: ~8 bytes (just scantime)

Total per timer: ~26 bytes
```

*Perfect for embedded systems with limited memory!*

---

## 🎓 How It Works

1. **`Update()`** - Reads current system time using `millis()`
2. **`Handle()`** - Compares elapsed time with configured intervals
3. **Status Flags** - Bit-packed `done` and `running` flags for efficiency
4. **No Interrupts** - Pure software-based polling (safer for beginners)

---

## ⚡ Performance Tips

- Call `Update()` every **1-10ms** for best accuracy
- Maximum delay: **4,294,967,295 ms** (~49 days)
- Can handle **unlimited timers** (limited only by memory)
- Resolution depends on your main loop frequency

---

## 🐛 Troubleshooting

### Timer not triggering?
- Make sure `Update()` is called regularly in your main loop
- Check that timer is enabled with `Enable()`

### Timer running too fast/slow?
- Adjust `Update()` call frequency
- Check system clock accuracy on your platform

### Multiple timers interfering?
- Each timer is independent - safe to use multiple timers simultaneously

---

## 📁 Project Structure

```
d:\SmartTimer\
├── README.md                 ← You are here
├── lib/
│   └── ScheduleTimer/
│       ├── ScheduleTimer.h
│       ├── ScheduleTimer.cpp
│       └── ...
└── examples/
    ├── example_led_blink.cpp
    ├── example_delayed_action.cpp
    └── ...
```

---

## 📄 License

MIT License
Copyright (c) 2025 Toon Automation

---

## 🤝 Contributing

Found a bug or want to improve? Feel free to contribute!

---

**Happy Timing! ⏱️**

*Last Updated: December 2025*
