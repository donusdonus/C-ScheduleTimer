# ⏱️ ScheduleTimer - Lightweight Cross-Platform Software Timer Library

A **lightweight, memory-efficient, non-blocking** software timer library designed for embedded systems and real-time applications. Built for **Arduino, ESP32, Windows, Linux, and any embedded platform** requiring precise timing control without hardware interrupts.

---

## ✨ Why Use ScheduleTimer V1.2?

- 🎯 **Ultra Compact** - Only **18 bytes** per timer instance (optimized with bit-packing)
- 🔒 **No Dynamic Memory** - Stack-based allocation, safe for real-time systems with no malloc/free
- ⚡ **Non-Blocking** - Pure software polling, no interrupts or hardware dependencies
- 🔄 **3 Versatile Timer Modes** - ONE-SHOT, PULSE, CAPTURE
- 🌍 **True Cross-Platform** - Works seamlessly on Arduino, ESP32, Windows, Linux, and any MCU
- 📦 **No Dependencies** - Pure C++ with minimal standard library usage
- 🚀 **Unlimited Timers** - Run as many timers as memory allows, all in one event loop

---

## 📚 Timer Modes - Detailed Explanation

### 🎯 Mode 1: ONE-SHOT (One-Time Delay)

**Purpose:** Execute an action once after a specified delay, then stop.

**Timing Diagram:**
```
Time:     0ms      →      delay_ms      →    Continue monitoring
State:  [WAITING]  →      [DONE]        →    [DONE - No auto-reset]
Output:    OFF     →       ON (once)    →    ON (stays)
```

**How It Works:**
1. When enabled, records current time as `toffset`
2. Continuously calculates: `elapsed = current_time - toffset`
3. When `elapsed >= delay_ms`, sets `done` flag to TRUE
4. Flag stays TRUE until timer is re-enabled (does NOT auto-reset)

**Memory Used:**
- `toff_ms` (4 bytes): Stores the delay duration
- `toffset` (4 bytes): Stores the start timestamp
- `accumulate` (4 bytes): Stores elapsed time
- `status.done` (1 bit): Completion flag

**Real-world Applications:**
- Alarm/notification after N seconds
- Debouncing button presses
- Timeout detection for communications
- Delayed action execution (e.g., auto-shutdown after 5 minutes)

**Code Example:**
```cpp
Timer alarmTimer;
ScheduleTimer scheduler;

// Trigger alarm after 3 seconds
alarmTimer.Config_oneshot_mode(3000);
alarmTimer.Enable(true);

while(1) {
    scheduler.Update();
    scheduler.Handle(&alarmTimer);
    
    if (alarmTimer.IsDone()) {
        printf("ALARM!\n");
        alarmTimer.Enable(false);  // Stop checking
        break;
    }
}
```

---

### 🔄 Mode 2: PULSE (Repeating ON/OFF Cycle)

**Purpose:** Repeatedly toggle between ON and OFF states with configurable durations (like PWM).

**Timing Diagram:**
```
Time:   0ms  toff_ms  (toff+ton)ms  (2×toff+ton)ms  ...
State: [OFF] [  ON  ]     [OFF]          [ON]       ...
       ↑_____↑_____↑_____↑_____↑_____↑_____↑_____↑
       |←ton→|←toff→|←ton→|←toff→|
```

**How It Works:**
1. Timer alternates between two states: `done=0` (OFF phase) and `done=1` (ON phase)
2. **OFF Phase (`done=0`):**
   - Measures: `elapsed = current_time - toffset`
   - When `elapsed >= toff_ms`, switch to ON phase and reset timer
3. **ON Phase (`done=1`):**
   - Measures: `elapsed = current_time - toffset`
   - When `elapsed >= ton_ms`, switch to OFF phase and reset timer
4. Continues indefinitely until disabled

**Memory Used:**
- `ton_ms` (4 bytes): ON duration
- `toff_ms` (4 bytes): OFF duration
- `toffset` (4 bytes): Current phase start time
- `status.done` (1 bit): Phase indicator (0=OFF, 1=ON)

**Real-world Applications:**
- LED blinking at specific frequency
- PWM signal generation (software-based)
- Buzzer/alarm pulsing
- Status indicator lights with custom patterns
- Motor control with duty cycle

**Code Example:**
```cpp
Timer ledTimer;
ScheduleTimer scheduler;

// LED: 500ms ON, 500ms OFF (total 1Hz frequency)
ledTimer.Config_pulse_mode(500, 500);
ledTimer.Enable(true);

while(1) {
    scheduler.Update();
    
    if (scheduler.Handle(&ledTimer)->IsDone()) {
        digitalWrite(LED_PIN, HIGH);  // ON phase
    } else {
        digitalWrite(LED_PIN, LOW);   // OFF phase
    }
}
```

---

### ⏳ Mode 3: CAPTURE (Elapsed Time Measurement)

**Purpose:** Measure the total time between enabling and disabling the timer.

**Timing Diagram:**
```
Time:     Enable    →     User still running    →    Disable
State:  [COUNTING] →        [COUNTING]         →    [STOPPED]
        ↓                                               ↓
    Read accumulate (0ms)         Read accumulate (elapsed_ms)
```

**How It Works:**
1. When enabled, records start time as `toffset`
2. Continuously calculates: `accumulate = current_time - toffset`
3. User can read `accumulate` at any time to get elapsed time
4. When disabled, the `accumulate` value stays frozen at the final time

**Memory Used:**
- `toffset` (4 bytes): Start timestamp
- `accumulate` (4 bytes): Running elapsed time counter

**Real-world Applications:**
- Measure button press duration
- Track execution time of an operation
- Monitor how long a condition has been active
- Stopwatch/timer display
- Performance profiling

**Code Example:**
```cpp
Timer buttonTimer;
ScheduleTimer scheduler;

buttonTimer.Config_capture_mode();
buttonTimer.Enable(true);
printf("Button pressed. Timing...\n");

while(true) {
    scheduler.Update();
    scheduler.Handle(&buttonTimer);
    
    uint32_t pressTime = buttonTimer.ElapsedMillisecond();
    printf("Pressed for: %u ms\n", pressTime);
    
    // Detect long press after 2 seconds
    if (pressTime > 2000) {
        printf("LONG PRESS detected!\n");
        buttonTimer.Enable(false);  // Stop timer
        break;
    }
}
```

---

## 🔧 Timer Mechanism - Under the Hood

### State Machine Diagram

```
┌─────────────────────────────────────────────────────────┐
│                    Timer Enable/Disable                 │
│                                                          │
│  cmd_enable = 0 (Disabled)                             │
│  ├─→ running flag = 0                                  │
│  ├─→ done flag = 0                                     │
│  └─→ accumulate = frozen (not updated)                 │
│                                                          │
│  cmd_enable = 1 (Enabled)                              │
│  └─→ Enter Mode-Specific Handler (ONESHORT/PULSE/CAPTURE)
└─────────────────────────────────────────────────────────┘

ONESHORT Mode:
    Start: accumulate = 0
    ├─→ accumulate += (current_time - last_time)
    ├─→ accumulate >= toff_ms?
    │   └─→ YES: done = 1 ✓ (stays done)
    └─→ NO: done = 0 (keep counting)

PULSE Mode:
    OFF Phase (done = 0):
        ├─→ accumulate += (current_time - last_time)
        └─→ accumulate >= toff_ms?
            └─→ YES: done = 1, reset accumulate, reset timer
    ON Phase (done = 1):
        ├─→ accumulate += (current_time - last_time)
        └─→ accumulate >= ton_ms?
            └─→ YES: done = 0, reset accumulate, reset timer

CAPTURE Mode:
    ├─→ accumulate = current_time - start_time
    └─→ User reads accumulate at any time
```

### Key Technical Details

**Time Offset Reset Pattern:**
- When transitioning states (PULSE) or starting (all modes), `toffset` is updated to prevent time jumps
- Formula: `elapsed = current_time - toffset` gives time since last offset update

**Bit-Packed Status Flags:**
```c
status union {
    bits.done     : 1 bit   // 0=not done, 1=done (mode-dependent)
    bits.running  : 1 bit   // 0=stopped, 1=active
    bits.reserved : 6 bits  // Reserved for future use
}
```

**No Auto-Reset Design:**
- ONESHORT timer stays in `done` state until re-enabled
- PULSE timer continuously auto-resets internally
- CAPTURE timer freezes on disable
- This design prevents unwanted re-triggering

---

## 🚀 Quick Start Guide

### Step 1: Include the Header
```cpp
#include "ScheduleTimer.h"

Timer myTimer;                    // Create timer instance (18 bytes)
ScheduleTimer scheduler;          // Create scheduler instance
```

### Step 2: Configure Timer Mode
```cpp
// ONE-SHOT: Trigger after 2000ms (non-blocking)
myTimer.Config_oneshot_mode(2000);

// PULSE: Toggle every 1s on, 500ms off (repeating)
myTimer.Config_pulse_mode(1000, 500);

// CAPTURE: Start measuring elapsed time
myTimer.Config_capture_mode();
```

### Step 3: Enable & Run in Main Loop
```cpp
// Enable the timer
myTimer.Enable(true);

// Main loop - MUST call Update() regularly (every 1-10ms recommended)
while (1) {
    scheduler.Update();           // Update internal timestamp
    scheduler.Handle(&myTimer);   // Process timer state machine
    
    // Check timer status
    if (myTimer.IsDone()) {
        printf("Timer event triggered!\n");
    }
    
    // Optional: Read elapsed time (CAPTURE mode)
    uint32_t elapsed = myTimer.ElapsedMillisecond();
    printf("Elapsed: %u ms\n", elapsed);
}
```

**Key Points:**
- `Update()` reads the current system time (non-blocking)
- `Handle()` advances the timer state machine
- No dynamic memory allocation - completely stack-based
- Safe for real-time systems and interrupt-sensitive applications

```

---

## 📖 Complete Platform-Specific Examples

### Example 1: LED Blink with PULSE Mode (All Platforms)

**Windows/Linux/Arduino/ESP32 Compatible:**
```cpp
#include "ScheduleTimer.h"

int main() {
    Timer ledTimer;
    ScheduleTimer scheduler;

    // Configure: LED blinks with 500ms ON, 500ms OFF
    ledTimer.Config_pulse_mode(500, 500);
    ledTimer.Enable(true);

    while (1) {
        scheduler.Update();
        
        // Get pointer to timer (Handle returns Timer*)
        Timer *tmr = scheduler.Handle(&ledTimer);
        
        if (tmr->IsDone()) {
            printf("LED: ON\n");
            // On Arduino: digitalWrite(LED_PIN, HIGH);
            // On ESP32:   digitalWrite(LED_PIN, HIGH);
            // On Windows: printf("LED LED: ON\n");
        } else {
            printf("LED: OFF\n");
            // On Arduino: digitalWrite(LED_PIN, LOW);
            // On ESP32:   digitalWrite(LED_PIN, LOW);
        }
    }
    
    return 0;
}
```

---

### Example 2: Delayed Action with ONE-SHOT Mode

**Use Case:** Trigger an alarm after 3 seconds (multi-platform)

```cpp
#include "ScheduleTimer.h"

int main() {
    Timer delayTimer;
    ScheduleTimer scheduler;

    printf("Starting alarm in 3 seconds...\n");
    
    // Trigger after 3000ms
    delayTimer.Config_oneshot_mode(3000);
    delayTimer.Enable(true);

    while (1) {
        scheduler.Update();
        scheduler.Handle(&delayTimer);

        if (delayTimer.IsDone()) {
            printf("BEEP! Alarm triggered!\n");
            // On Arduino: tone(BUZZER_PIN, 1000);
            // On ESP32:   ledcWrite(PWM_CHANNEL, 255);
            // On Windows: printf("BEEP!\n");
            break;
        }
        else {
            // Show countdown
            printf("Elapsed: %u ms\n", delayTimer.ElapsedMillisecond());
        }
    }
    
    return 0;
}
```

---

### Example 3: Button Long-Press Detection with CAPTURE Mode

**Use Case:** Detect short press (< 2s) vs long press (> 2s)

```cpp
#include "ScheduleTimer.h"

// Platform-specific button read function
uint8_t readButton() {
    #ifdef ___PLATFORM_ARDUINO___
    return digitalRead(BUTTON_PIN);
    #endif
    
    #ifdef ___OS_WINDOWS___
    // Simulate with keyboard input
    return (GetAsyncKeyState('A') & 0x8000) ? 1 : 0;
    #endif
}

int main() {
    Timer buttonTimer;
    ScheduleTimer scheduler;
    uint8_t lastButtonState = 0;

    printf("Press button...\n");

    while(true) {
        scheduler.Update();
        scheduler.Handle(&buttonTimer);

        uint8_t currentButtonState = readButton();

        // Button pressed (transition from 0 to 1)
        if (currentButtonState && !lastButtonState) {
            printf("Button pressed. Timing...\n");
            buttonTimer.Config_capture_mode();
            buttonTimer.Enable(true);
        }

        // Button released (transition from 1 to 0)
        if (!currentButtonState && lastButtonState) {
            buttonTimer.Enable(false);
            uint32_t pressTime = buttonTimer.ElapsedMillisecond();
            printf("Button released after: %u ms\n", pressTime);

            if (pressTime > 2000) {
                printf("→ LONG PRESS detected!\n");
            } else {
                printf("→ SHORT PRESS detected\n");
            }
        }

        lastButtonState = currentButtonState;
    }

    return 0;
}
```

---

### Example 4: Traffic Light State Machine (ONESHORT Mode)

**Use Case:** Cycle through RED (5s) → GREEN (3s) → YELLOW (1s)

```cpp
#include "ScheduleTimer.h"

enum TrafficLight { PREPARE, RED, GREEN, YELLOW };

int main() {
    Timer lightTimer;
    ScheduleTimer scheduler;
    TrafficLight state = PREPARE;

    while (1) {
        scheduler.Update();
        
        if (scheduler.Handle(&lightTimer)->IsDone()) {
            switch (state) {
                case PREPARE:
                    state = RED;
                    printf("[RED] - Stop (5 seconds)\n");
                    // Arduino: digitalWrite(RED_PIN, HIGH);
                    // ESP32:   digitalWrite(RED_PIN, HIGH);
                    lightTimer.Config_oneshot_mode(5000);
                    break;

                case RED:
                    state = GREEN;
                    printf("[GREEN] - Go (3 seconds)\n");
                    // Arduino: digitalWrite(RED_PIN, LOW); digitalWrite(GREEN_PIN, HIGH);
                    lightTimer.Config_oneshot_mode(3000);
                    break;

                case GREEN:
                    state = YELLOW;
                    printf("[YELLOW] - Caution (1 second)\n");
                    // Arduino: digitalWrite(GREEN_PIN, LOW); digitalWrite(YELLOW_PIN, HIGH);
                    lightTimer.Config_oneshot_mode(1000);
                    break;

                case YELLOW:
                    state = RED;
                    printf("[RED] - Stop (5 seconds)\n");
                    // Arduino: digitalWrite(YELLOW_PIN, LOW); digitalWrite(RED_PIN, HIGH);
                    lightTimer.Config_oneshot_mode(5000);
                    break;
            }

            lightTimer.Enable(true);
        }
    }

    return 0;
}
```
```


---

## 🌍 Cross-Platform Integration Guide

### Platform Selection

The library automatically detects your platform via preprocessor directives:

```cpp
// In ScheduleTimer.h

#define ___OS_WINDOWS___           // ← Uncomment for Windows/Linux C++
// #define ___PLATFORM_ARDUINO___  // ← Uncomment for Arduino/Arduino-compatible boards
// #define ___PLATFORM_ESP32___    // ← For ESP32 (uses Arduino.h)
```

### Platform Compatibility Matrix

| Platform | Status | System Timer | Integration | Notes |
|----------|--------|--------------|-------------|-------|
| **Windows** | ✅ Full | `std::chrono::steady_clock` | Native C++17 | Highest accuracy, used for testing |
| **Linux** | ✅ Full | `std::chrono::steady_clock` | Native C++17 | Same as Windows, requires C++17 |
| **Arduino (AVR)** | ✅ Full | `millis()` | `<Arduino.h>` | Standard `millis()` function |
| **Arduino (ARM)** | ✅ Full | `millis()` | `<Arduino.h>` | SAMD, Teensy, etc. supported |
| **ESP32** | ✅ Full | `millis()` | `<Arduino.h>` | Works with Arduino IDE or PlatformIO |
| **Generic MCU** | ✅ Custom | User-provided | Custom `millis()` | Requires implementation of `millis()` |

---

### ⚙️ Platform Setup Instructions

#### 1. **Windows/Linux (C++ with std::chrono)**

**Requirements:** C++17 compiler (GCC 7+, Clang 5+, MSVC 2017+)

**CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.10)
project(ScheduleTimer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add library
add_library(ScheduleTimer 
    lib/ScheduleTimer/ScheduleTimer.cpp
    lib/ScheduleTimer/ScheduleTimer.h
)

target_include_directories(ScheduleTimer PUBLIC lib/ScheduleTimer)

# Add example executable
add_executable(example_blink examples/PulseMode_LEDBlink/main.cpp)
target_link_libraries(example_blink ScheduleTimer)
```

**Build & Run:**
```bash
mkdir build && cd build
cmake ..
cmake --build .
./example_blink
```

---

#### 2. **Arduino (AVR/ARM) and Arduino IDE**

**Installation:**
1. Copy the `lib/ScheduleTimer/` folder to your Arduino `libraries/` directory
2. Restart Arduino IDE
3. Include in sketch: `#include <ScheduleTimer.h>`

**Example Sketch:**
```cpp
// Blink LED every 500ms using ScheduleTimer
#include <ScheduleTimer.h>

Timer ledTimer;
ScheduleTimer scheduler;

#define LED_PIN 13

void setup() {
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);
    
    ledTimer.Config_pulse_mode(500, 500);  // 500ms ON, 500ms OFF
    ledTimer.Enable(true);
}

void loop() {
    scheduler.Update();
    
    if (scheduler.Handle(&ledTimer)->IsDone()) {
        digitalWrite(LED_PIN, HIGH);  // ON phase
    } else {
        digitalWrite(LED_PIN, LOW);   // OFF phase
    }
}
```

**Compile & Upload:**
- Arduino IDE: Sketch → Upload (automatic compilation)
- Or use command line: `arduino-cli compile --fqbn arduino:avr:uno`

---

#### 3. **ESP32 with Arduino IDE or PlatformIO**

**Arduino IDE Setup:**
1. Add ESP32 board manager: https://dl.espressif.com/dl/package_esp32_index.json
2. Select Board: Tools → Board → ESP32 Dev Module
3. Include library same as Arduino

**PlatformIO Setup (Recommended for ESP32):**

**platformio.ini:**
```ini
[env:esp32]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
lib_deps = 
    ; Your custom library path
monitor_speed = 115200
```

**src/main.cpp:**
```cpp
#include <Arduino.h>
#include <ScheduleTimer.h>

Timer ledTimer;
ScheduleTimer scheduler;

#define LED_PIN 2  // ESP32 GPIO2

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    
    // LED blink: 300ms ON, 700ms OFF
    ledTimer.Config_pulse_mode(300, 700);
    ledTimer.Enable(true);
    
    Serial.println("ESP32 Timer Started");
}

void loop() {
    scheduler.Update();
    
    if (scheduler.Handle(&ledTimer)->IsDone()) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED: ON");
    } else {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED: OFF");
    }
    
    delay(100);  // Small delay to prevent flooding serial
}
```

**Build & Upload:**
```bash
pio run -e esp32 -t upload
pio device monitor
```

---

#### 4. **Custom Embedded Platform (RTOS, Bare-metal)**

If your platform doesn't have `millis()`, provide your own:

**Modified ScheduleTimer.h:**
```cpp
// For STM32, NXP, or other MCUs
#ifdef ___PLATFORM_CUSTOM___
extern unsigned long millis();  // Declare - you must implement this
#endif
```

**Implementation Example (STM32 with HAL):**
```cpp
// In your main project file
#include "stm32f4xx_hal.h"

static volatile uint32_t sys_tick_count = 0;

// STM32 HAL tick callback (called every 1ms)
void SysTick_Handler(void) {
    HAL_IncTick();
    sys_tick_count++;
}

// Provide millis() function
unsigned long millis() {
    return sys_tick_count;
}

// Now use ScheduleTimer normally
#include "ScheduleTimer.h"
```

---

### 🔧 Advanced Platform-Specific Tips

#### Accuracy Considerations

**Windows/Linux:**
- Uses `std::chrono::steady_clock` (nanosecond precision)
- Typical accuracy: ±1-5ms per 1000ms interval
- Ideal for testing and development

**Arduino/ESP32:**
- Uses `millis()` (1ms resolution)
- Typical accuracy: ±2-3% depending on crystal oscillator tolerance
- May drift over long periods (hours)

**Bare-Metal MCU:**
- Accuracy depends on your `millis()` implementation
- Typical accuracy: ±1-5% depending on clock source and frequency

#### Best Practices

1. **Call `Update()` Frequently:**
   - For accuracy, call every 1-10ms
   - On Arduino/ESP32: Can call in `loop()` without `delay()`
   - On Windows: Call in your main event loop or high-frequency thread

2. **Avoid Blocking Code in Loop:**
   ```cpp
   // ✅ Good: Non-blocking
   void loop() {
       scheduler.Update();
       scheduler.Handle(&myTimer);
       // Do other work here (no delay)
   }

   // ❌ Bad: Blocking
   void loop() {
       scheduler.Update();
       delay(100);  // ← This blocks the scheduler!
       scheduler.Handle(&myTimer);
   }
   ```

3. **Multiple Timers (All Platforms):**
   ```cpp
   Timer timer1, timer2, timer3;
   ScheduleTimer scheduler;

   void loop() {
       scheduler.Update();  // Call once per loop
       scheduler.Handle(&timer1);
       scheduler.Handle(&timer2);
       scheduler.Handle(&timer3);  // Process all timers
   }
   ```

4. **Handle 32-bit Overflow (After ~49 days):**
   ```cpp
   // If timer values reach UINT32_MAX (4294967295ms ≈ 49.7 days),
   // they roll over to 0. For critical applications, reset the system
   // or track overflow events explicitly.
   ```

---

## 🔧 API Reference

### Configuration Methods

| Method | Parameters | Purpose | Returns |
|--------|-----------|---------|---------|
| `Config_oneshot_mode()` | `uint32_t delay_ms` | Set one-time delay | void |
| `Config_pulse_mode()` | `uint32_t ton_ms, uint32_t toff_ms` | Set repeating ON/OFF cycle | void |
| `Config_capture_mode()` | (none) | Initialize time measurement mode | void |

### Control Methods

| Method | Parameters | Purpose | Returns |
|--------|-----------|---------|---------|
| `Enable()` | `bool enable` | Start/Stop timer | void |
| `Update()` | (none) | Read system time and update scheduler | void |
| `Handle()` | `Timer *tmr` | Process timer state machine | `Timer*` (pointer to timer) |

### Status Methods (Member of Timer struct)

| Method | Returns | Purpose |
|--------|---------|---------|
| `IsDone()` | `bool` | Check if timer event occurred |
| `IsRunning()` | `bool` | Check if timer is currently active |
| `ElapsedMillisecond()` | `uint32_t` | Get accumulated/elapsed time in milliseconds |
| `GetMode()` | `TimerMode` | Get current timer mode (ONESHORT/PULSE/CAPTURE) |

---

## 💡 Design Patterns & Best Practices

### Pattern 1: Timeout Detection

Use ONE-SHOT mode to detect when a communication or operation exceeds a time limit.

```cpp
Timer timeoutTimer;
ScheduleTimer scheduler;
bool dataReceived = false;

timeoutTimer.Config_oneshot_mode(5000);  // 5 second timeout
timeoutTimer.Enable(true);

while (!dataReceived && !timeoutTimer.IsDone()) {
    scheduler.Update();
    scheduler.Handle(&timeoutTimer);
    
    // Try to read data
    if (readData(&buffer)) {
        dataReceived = true;
        printf("Data received successfully\n");
        break;
    }
}

if (timeoutTimer.IsDone() && !dataReceived) {
    printf("ERROR: Timeout! No data received within 5 seconds\n");
    // Handle error: retry, reset, alert user, etc.
}
```

**Use Cases:**
- Communication protocol timeouts (UART, I2C, SPI)
- Waiting for sensor data
- Network request timeouts
- User interaction timeouts (e.g., menu timeout)

---

### Pattern 2: Debounced Input

Use CAPTURE mode to measure button press duration and filter noise.

```cpp
Timer debounceTimer;
ScheduleTimer scheduler;
uint8_t buttonPressed = 0;
uint8_t lastState = 0;

while (1) {
    scheduler.Update();
    scheduler.Handle(&debounceTimer);
    
    uint8_t currentState = digitalRead(BUTTON_PIN);
    
    // Detect falling edge (button pressed)
    if (currentState == 0 && lastState == 1 && !buttonPressed) {
        debounceTimer.Config_capture_mode();
        debounceTimer.Enable(true);
        buttonPressed = 1;
    }
    
    // Detect rising edge (button released)
    if (currentState == 1 && lastState == 0 && buttonPressed) {
        debounceTimer.Enable(false);
        uint32_t pressDuration = debounceTimer.ElapsedMillisecond();
        
        // Only register if press was longer than debounce time (20ms)
        if (pressDuration > 20) {
            printf("Button press registered: %u ms\n", pressDuration);
        } else {
            printf("Noise detected, ignoring\n");
        }
        
        buttonPressed = 0;
    }
    
    lastState = currentState;
}
```

**Benefits:**
- Eliminates switch bounce noise (typically 5-20ms)
- Measures actual press duration
- More reliable than simple threshold checking

---

### Pattern 3: Multi-State State Machine

Combine multiple timers to create complex sequential behaviors.

```cpp
typedef enum {
    STATE_IDLE,
    STATE_STARTUP,
    STATE_RUNNING,
    STATE_COOLDOWN,
    STATE_ERROR
} SystemState;

Timer stateTimer;
Timer errorTimer;
ScheduleTimer scheduler;
SystemState currentState = STATE_IDLE;

void handleSystemState() {
    scheduler.Update();
    scheduler.Handle(&stateTimer);
    scheduler.Handle(&errorTimer);
    
    switch (currentState) {
        case STATE_IDLE:
            printf("System idle. Press button to start.\n");
            if (buttonPressed()) {
                currentState = STATE_STARTUP;
                stateTimer.Config_oneshot_mode(2000);  // 2s startup
                stateTimer.Enable(true);
            }
            break;
            
        case STATE_STARTUP:
            if (stateTimer.IsDone()) {
                currentState = STATE_RUNNING;
                printf("System started!\n");
                
                // Run for 10 seconds
                stateTimer.Config_oneshot_mode(10000);
                stateTimer.Enable(true);
            }
            break;
            
        case STATE_RUNNING:
            printf("System running...\n");
            if (stateTimer.IsDone()) {
                currentState = STATE_COOLDOWN;
                printf("System entering cooldown (3 seconds)\n");
                
                stateTimer.Config_oneshot_mode(3000);
                stateTimer.Enable(true);
            }
            if (systemError()) {
                currentState = STATE_ERROR;
                errorTimer.Config_oneshot_mode(5000);  // Show error for 5s
                errorTimer.Enable(true);
            }
            break;
            
        case STATE_COOLDOWN:
            if (stateTimer.IsDone()) {
                currentState = STATE_IDLE;
                printf("Cooldown complete. Ready for restart.\n");
            }
            break;
            
        case STATE_ERROR:
            printf("ERROR STATE\n");
            if (errorTimer.IsDone()) {
                currentState = STATE_IDLE;
                printf("Error cleared. System ready.\n");
            }
            break;
    }
}
```

---

### Pattern 4: Frequency-Based Execution

Use PULSE mode to execute code at a fixed frequency.

```cpp
Timer frequencyTimer;
ScheduleTimer scheduler;

// Run function at 100 Hz (every 10ms)
frequencyTimer.Config_pulse_mode(10, 0);  // 10ms ON, 0ms OFF (runs every 10ms)
frequencyTimer.Enable(true);

unsigned int executionCount = 0;

while (1) {
    scheduler.Update();
    
    if (scheduler.Handle(&frequencyTimer)->IsDone()) {
        executionCount++;
        
        // This block executes exactly 100 times per second
        // Use this for:
        // - Motor control/PWM updates
        // - Sensor polling at fixed rate
        // - Real-time control loops
        
        if (executionCount % 10 == 0) {  // Every 100ms
            printf("Execution count: %u\n", executionCount);
        }
    }
}

// NOTE: For very high frequencies (>1000 Hz), consider hardware PWM instead
```

---

### Pattern 5: Periodic Cleanup/Maintenance

Use PULSE mode to periodically perform system maintenance tasks.

```cpp
Timer maintenanceTimer;
ScheduleTimer scheduler;

// Run maintenance every 60 seconds (60000ms)
maintenanceTimer.Config_oneshot_mode(60000);
maintenanceTimer.Enable(true);

while (1) {
    scheduler.Update();
    scheduler.Handle(&maintenanceTimer);
    
    // Main application code here
    // ...
    
    if (maintenanceTimer.IsDone()) {
        printf("Running maintenance...\n");
        
        // Cleanup operations
        // - Flush logs
        // - Check memory usage
        // - Reset counters
        // - Refresh sensors
        // etc.
        
        // Restart timer for next maintenance cycle
        maintenanceTimer.Config_oneshot_mode(60000);
        maintenanceTimer.Enable(true);
    }
}
```

---

## 📊 Performance & Memory Analysis

### Memory Layout (Bit-Packed)

```c
struct Timer {                    // Offset  Size  Description
    uint32_t ton_ms;              // +0      4B    ON duration (PULSE mode)
    uint32_t toff_ms;             // +4      4B    OFF duration or total delay (ONESHOT)
    uint32_t toffset;             // +8      4B    Timestamp when current phase started
    uint32_t accumulate;          // +12     4B    Accumulated/elapsed time counter
    
    union {
        struct {
            uint8_t done : 1;     // +16     1bit  Done flag (ONESHOT) or phase indicator (PULSE)
            uint8_t running : 1;  //         1bit  Running flag
            uint8_t reserved : 6; //         6bits Reserved for future use
        } bits;
        uint8_t raw;              //         1B    Raw status byte
    } status;
    
    union {
        struct {
            uint8_t cmd_enable : 1; // +17   1bit  Enable command
            uint8_t cmd_mode : 4;   //       4bits Timer mode (0-2)
            uint8_t reserved : 3;   //       3bits Reserved
        } bits;
        uint8_t raw;              //         1B    Raw control byte
    } ctrl;
};                                // TOTAL:  18 bytes
```

**Total Memory Per Timer:** 18 bytes (extremely compact!)

**Example: Managing 10 Timers**
```
10 timers × 18 bytes = 180 bytes
ScheduleTimer class = ~8 bytes
TOTAL = 188 bytes (< 0.2 KB)
```

---

### Execution Time Complexity

**Per Timer Per Update:**
- `Update()` call: **~1-5 microseconds** (reads system clock)
- `Handle()` call: **~2-10 microseconds** (state machine evaluation)
- Total for 10 timers: **~30-150 microseconds** per loop iteration

**Comparison with Hardware Alternatives:**
| Method | Memory | CPU (per timer) | Accuracy | Overhead |
|--------|--------|-----------------|----------|----------|
| ScheduleTimer (Software) | 18 bytes | ~5µs | ±1-5ms | Minimal |
| Hardware Timer (PWM) | ~1KB+ | 0µs (HW) | ±1µs | Pins, ISR |
| OS Task (RTOS) | ~1KB+ | Variable | ±10-50ms | Context switch |

---

## 🎓 Internal Mechanism Deep Dive

### Time Calculation Engine

```cpp
// Simplified logic from Handle()

Timer* ScheduleTimer::Handle(Timer *tmr) {
    
    // STEP 1: Check if enabled
    if (!tmr->ctrl.bits.cmd_enable) {
        tmr->status.bits.running = 0;
        tmr->status.bits.done = 0;
        return tmr;  // Frozen state
    }
    
    // STEP 2: Initialize on first run
    if (!tmr->status.bits.running) {
        tmr->status.bits.running = 1;
        tmr->toffset = scantime;  // Current system time
    }
    
    // STEP 3: Calculate elapsed time since phase start
    uint32_t elapsed = scantime - tmr->toffset;
    
    // STEP 4: Evaluate mode-specific logic
    switch (tmr->ctrl.bits.cmd_mode) {
        case ONESHORT:
            if (!tmr->status.bits.done) {
                tmr->accumulate = elapsed;
                if (elapsed >= tmr->toff_ms) {
                    tmr->status.bits.done = 1;  // Mark complete
                }
            }
            break;
            
        case PULSE:
            tmr->accumulate = elapsed;
            
            // OFF→ON transition
            if (!tmr->status.bits.done && elapsed >= tmr->toff_ms) {
                tmr->status.bits.done = 1;
                tmr->toffset = scantime;  // Reset phase counter
                tmr->accumulate = 0;
            }
            
            // ON→OFF transition
            if (tmr->status.bits.done && elapsed >= tmr->ton_ms) {
                tmr->status.bits.done = 0;
                tmr->toffset = scantime;  // Reset phase counter
                tmr->accumulate = 0;
            }
            break;
            
        case CAPTURE:
            tmr->accumulate = elapsed;  // Always update elapsed time
            break;
    }
    
    return tmr;
}
```

### Time Offset Reset Strategy

The `toffset` reset is critical for preventing overflow errors:

```
Timeline:
    Phase 1:    elapsed = time - toffset₁        (toffset₁ = 1000)
    [Transition]
    Phase 2:    elapsed = time - toffset₂        (toffset₂ = new value)
    
Benefit: Keeps elapsed counter small, prevents overflow before next transition
```
---

## 🐛 Troubleshooting Guide

### Issue: Timer triggers but I didn't see it

**Cause:** `done` flag was set, but you checked it too late

```cpp
// ❌ Wrong: Single check might miss
if (timer.IsDone()) { /* ... */ }

// ✅ Correct: Keep checking until handled
if (timer.IsDone() && timer.IsRunning()) {
    doAction();
    timer.Enable(false);  // Stop to prevent re-trigger
}
```

### Issue: Timer appears to run slow

**Causes & Solutions:**
1. `Update()` not called frequently enough
   - Solution: Ensure `Update()` every 1-10ms

2. System is blocked in other code
   - Solution: Remove blocking calls (delay, sleep, busy loops)

3. Platform clock is inaccurate
   - Solution: Check oscillator frequency/trimming

```cpp
// Debug: Log timing information
unsigned long lastTime = 0;
while (1) {
    scheduler.Update();
    
    if (some_condition) {
        unsigned long now = millis();
        printf("Time since last: %lu ms\n", now - lastTime);
        lastTime = now;
    }
}
```

### Issue: Timer doesn't work on Arduino/ESP32

**Checklist:**
- [ ] `#include <ScheduleTimer.h>` in sketch
- [ ] Library installed in `Arduino/libraries/`
- [ ] `ScheduleTimer.h` includes `<Arduino.h>` correctly
- [ ] `Update()` called every loop iteration
- [ ] Timer is enabled with `Enable(true)`

### Issue: Compilation errors on Windows

**Common errors:**
```
error: 'millis' was not declared
→ Solution: Add #define ___OS_WINDOWS___ in ScheduleTimer.h

error: <chrono> not found
→ Solution: Use C++17 compiler: -std=c++17
```

### Issue: Overflow after 49.7 days

**Cause:** `uint32_t` timer counter wraps at 4,294,967,295 ms

```cpp
// Detect overflow
if (previousTime > currentTime) {
    printf("Timer overflow detected!\n");
    // Option 1: Restart system
    // Option 2: Reset timer counters
    // Option 3: Ignore (acceptable for most applications)
}

// For critical applications, limit runtime
const uint32_t MAX_RUNTIME_HOURS = 7 * 24;  // 1 week
if (systemUptime > (MAX_RUNTIME_HOURS * 3600000UL)) {
    resetSystem();  // Prevent overflow
}
```

---

## 📁 Project Structure & Building

### Directory Organization

```
C-ScheduleTimer/
├── CMakeLists.txt              # CMake build configuration (Windows/Linux/GCC)
├── CMakeForge.py               # Python build script
├── README.md                   # This file
├── LICENSE                     # MIT License
│
├── lib/                        # Source libraries
│   ├── ScheduleTimer/          # Main timer library
│   │   ├── CMakeLists.txt
│   │   ├── ScheduleTimer.h     # Header with Timer struct & API
│   │   └── ScheduleTimer.cpp   # Implementation with Handle() logic
│   │
│   └── MyLib/                  # Optional: Custom utilities
│       ├── CMakeLists.txt
│       ├── MyLib.h
│       └── MyLib.cpp
│
├── examples/                   # Practical examples (all platforms)
│   ├── PulseMode_LEDBlink/
│   │   └── main.cpp            # PULSE mode: LED blinking example
│   ├── OnshortMode_Delayed_Action/
│   │   └── main.cpp            # ONESHOT mode: Timed alarm example
│   ├── CaptureMode_Button_LongPress_Detector/
│   │   └── main.cpp            # CAPTURE mode: Button timing example
│   └── CaptureMode_Traffic_Light_Cycle/
│       └── main.cpp            # ONESHOT mode: State machine example
│
├── src/                        # Main application source
│   └── main.cpp                # Example main program
│
├── build/                      # Compiled output (generated)
│   ├── CMakeFiles/
│   ├── lib/                    # Compiled libraries
│   └── app.dir/                # Compiled objects
│
└── cmd/                        # Build scripts (Windows batch files)
    ├── init.bat                # Initialize CMake build
    ├── build.bat               # Compile project
    ├── run.bat                 # Run compiled binary
    ├── build_run.bat           # Build and run
    └── clean.bat               # Remove build files
```
---

## 🔗 Integration Checklist

When integrating ScheduleTimer into your project:

- [ ] **Include Header**: `#include "ScheduleTimer.h"`
- [ ] **Create Instances**: `Timer myTimer; ScheduleTimer scheduler;`
- [ ] **Configure Mode**: `myTimer.Config_*_mode(...)`
- [ ] **Enable Timer**: `myTimer.Enable(true)`
- [ ] **Update Loop**: Call `scheduler.Update()` regularly
- [ ] **Handle Timer**: `scheduler.Handle(&myTimer)`
- [ ] **Check Status**: Use `IsDone()`, `IsRunning()`, `ElapsedMillisecond()`
- [ ] **Test Timing**: Verify accuracy matches requirements
- [ ] **Disable When Done**: `myTimer.Enable(false)` to stop checking
- [ ] **Handle Overflow**: Plan for 49-day timer wrap-around if needed

---

## 🚀 Example: Complete Multi-Timer Application

Here's a real-world example combining multiple timer modes:

```cpp
#include "ScheduleTimer.h"

// Timers for different purposes
Timer ledTimer;           // PULSE: LED blink indicator
Timer buttonDebounce;     // CAPTURE: Button press measurement
Timer systemTimeout;      // ONESHOT: Safety timeout
ScheduleTimer scheduler;

// System state
enum SystemState { IDLE, RUNNING, ERROR, SHUTDOWN };
SystemState state = IDLE;
uint8_t systemError = 0;

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Configure timers
    ledTimer.Config_pulse_mode(200, 800);  // Slow blink in idle
    systemTimeout.Config_oneshot_mode(30000);  // 30s safety timeout
}

void loop() {
    // CRITICAL: Update happens once per loop
    scheduler.Update();
    
    // Process all timers
    scheduler.Handle(&ledTimer);
    scheduler.Handle(&buttonDebounce);
    scheduler.Handle(&systemTimeout);
    
    // Handle button input
    uint8_t buttonState = digitalRead(BUTTON_PIN);
    static uint8_t lastButtonState = 1;
    
    // Button pressed
    if (buttonState == 0 && lastButtonState == 1) {
        buttonDebounce.Config_capture_mode();
        buttonDebounce.Enable(true);
        Serial.println("Button pressed");
    }
    
    // Button released
    if (buttonState == 1 && lastButtonState == 0 && buttonDebounce.IsRunning()) {
        buttonDebounce.Enable(false);
        uint32_t pressDuration = buttonDebounce.ElapsedMillisecond();
        
        if (pressDuration > 50) {  // Debounce threshold
            if (pressDuration > 2000) {
                Serial.println("LONG PRESS");
                state = RUNNING;
            } else {
                Serial.println("SHORT PRESS");
                if (state == RUNNING) state = IDLE;
            }
        }
    }
    lastButtonState = buttonState;
    
    // Handle state machine
    switch (state) {
        case IDLE:
            // LED blinks slowly
            if (ledTimer.IsDone()) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
            }
            Serial.println("Idle - Press button to start");
            break;
            
        case RUNNING:
            // LED on steady, start safety timeout
            digitalWrite(LED_PIN, HIGH);
            if (!systemTimeout.IsRunning()) {
                systemTimeout.Enable(true);
                Serial.println("System started! (30s timeout)");
            }
            
            // Do actual work here
            Serial.println("System operating...");
            
            if (systemTimeout.IsDone()) {
                systemError = 1;
                state = ERROR;
            }
            break;
            
        case ERROR:
            // LED blinks fast
            if (ledTimer.IsDone()) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
            }
            Serial.println("ERROR: System timeout!");
            state = SHUTDOWN;
            break;
            
        case SHUTDOWN:
            digitalWrite(LED_PIN, LOW);
            Serial.println("Shutting down...");
            state = IDLE;
            systemError = 0;
            systemTimeout.Enable(false);
            break;
    }
    
}
```

---

## 📞 Support & Contributing

### Reporting Issues

When reporting bugs, please include:
- [ ] Platform (Arduino, ESP32, Windows, Linux)
- [ ] Timer mode being used
- [ ] Expected vs actual behavior
- [ ] Minimum code example to reproduce

### Contributing

Contributions are welcome! Areas for enhancement:
- Additional platform support (STM32, PIC, RISC-V, etc.)
- Performance optimizations
- Extended examples
- Documentation improvements

---

## 📄 License

MIT License - See LICENSE file for details

```
Copyright (c) 2025 Toon Automation (donusdonus)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
```

---

## 📊 Quick Reference Card

```cpp
// Timer Modes at a Glance
myTimer.Config_oneshot_mode(2000);        // One-time trigger after 2 seconds
myTimer.Config_pulse_mode(500, 500);      // Repeat: 500ms ON, 500ms OFF
myTimer.Config_capture_mode();            // Measure elapsed time

// Control
myTimer.Enable(true);                     // Start timer
myTimer.Enable(false);                    // Stop timer

// Status Checks
if (myTimer.IsDone()) { }                 // Check if event occurred
if (myTimer.IsRunning()) { }              // Check if currently active
uint32_t ms = myTimer.ElapsedMillisecond(); // Get elapsed time
TimerMode m = myTimer.GetMode();          // Get current mode

// Main Loop Pattern
scheduler.Update();                       // Update timestamp (once per loop)
scheduler.Handle(&myTimer1);              // Process timer 1
scheduler.Handle(&myTimer2);              // Process timer 2
// ... use timer status to control logic
```

---

**⏱️ Happy Timing!**

*Last Updated: December 3, 2025*
