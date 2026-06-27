#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// BERC4833 Real Time Systems — Assignment
// Title   : Medical Infusion Pump – Dose Control
// MCU     : ESP32 (Wokwi simulation)
// RTOS    : FreeRTOS — Rate Monotonic Scheduling (RMS)
// Version : v4
//
// ── SCHEDULING MODE ─────────────────────────────────────────
//   CONTENTION_DEMO = 1  →  PROBLEM: Motor acquires i2cMutex
//                            every 10 ms step. LCD holds it for
//                            100 ms → up to 10 missed steps per
//                            LCD cycle (original scheduling problem).
//
//   CONTENTION_DEMO = 0  →  SOLUTION: Motor enable pin state is
//                            cached. i2cMutex only acquired on
//                            infusion state CHANGE (button press —
//                            infrequent). Per-step I2C contention
//                            is eliminated entirely.
// ─────────────────────────────────────────────────────────────
#define CONTENTION_DEMO  1   // Set 0 to run SOLUTION mode

// ── RMS Priority Assignment ───────────────────────────────────
// Strict RMS rule: shorter period → higher FreeRTOS priority number
//
//   Motor   T=10  ms → P=4  ← HIGHEST periodic task (RMS-correct)
//   Button  T=20  ms → P=3  ← RMS-ordered
//   Dose    T=250 ms → P=2
//   LCD     T=500 ms → P=1
//   Monitor T=5000ms → P=0
//
// SAFETY OVERRIDE: Button assigned P=5 (above RMS band) to guarantee
// <20 ms emergency-stop latency regardless of motor-step preemption.
// Documented deviation — does NOT invalidate Liu & Layland bound:
// U_btn = <1ms / 20ms = 0.05 (negligible). Full justification in report.
// ─────────────────────────────────────────────────────────────

// ── RMS Response Time Analysis ────────────────────────────────
// (Iterative method, RMS priority order)
//   R_Motor  : R₀=2, R₁=2+⌈2/20⌉·1=3 ms             < T=10 ms  ✓
//   R_Dose   : R₀=40 → converges at R≈59 ms          < T=250 ms ✓
//   R_LCD    : R₀=100 → converges at R≈260 ms         < T=500 ms ✓
// All tasks meet deadlines under RMS. See Part 1 report for full table.
// ─────────────────────────────────────────────────────────────

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define BUTTON_PIN  13
#define MOTOR_PIN   18
#define LED_ALARM    2
#define LED_ACTIVE  19

// ============================================================
// HARDWARE
// ============================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================================
// RTOS OBJECTS
// ============================================================
SemaphoreHandle_t i2cMutex;

TaskHandle_t hButton  = NULL;
TaskHandle_t hDose    = NULL;
TaskHandle_t hMotor   = NULL;
TaskHandle_t hLCD     = NULL;
TaskHandle_t hMonitor = NULL;

// ============================================================
// SHARED SYSTEM STATE  (volatile — cross-task access)
// ============================================================
volatile bool  infusion_active     = true;
volatile bool  infusion_prev_state = true;   // SOLUTION: track for enable-pin updates
volatile float dose_volume_ml      = 0.0f;
volatile bool  alarm_flag          = false;

// ============================================================
// RUNTIME COUNTERS
// ============================================================
volatile uint32_t btnTask_exec   = 0, btnTask_miss   = 0;
volatile uint32_t doseTask_exec  = 0, doseTask_miss  = 0;
volatile uint32_t motorTask_exec = 0, motorTask_miss = 0;
volatile uint32_t lcdTask_exec   = 0, lcdTask_miss   = 0;
volatile uint32_t lcdTask_skip   = 0;   // LCD frames dropped due to mutex timeout
volatile uint32_t monitorTask_exec = 0;

// ============================================================
// HELPER — busy-wait CPU workload simulation
// ============================================================
static void do_work(uint32_t ms) {
    volatile unsigned long dummy = 0;
    unsigned long t = millis();
    while ((millis() - t) < ms) dummy++;
}

// ============================================================
// TASK 1 — Button Monitor
// Period : 20 ms | WCET : <1 ms | Priority : 5 (SAFETY OVERRIDE)
//
// Edge-triggered. 1st press → emergency stop + alarm latch.
//                 2nd press → operator reset, alarm clear.
// Assigned P=5 (above RMS band) — safety override. See note above.
// ============================================================
void taskButton(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);
    bool prev_pressed = false;

    while (1) {
        TickType_t deadline = last_wake + period;

        bool pressed = (digitalRead(BUTTON_PIN) == LOW);
        bool edge    = pressed && !prev_pressed;
        prev_pressed = pressed;

        if (edge) {
            if (infusion_active) {
                infusion_active = false;
                alarm_flag      = true;
                digitalWrite(LED_ALARM,  HIGH);
                digitalWrite(LED_ACTIVE, LOW);
                Serial.println("[BTN] *** EMERGENCY STOP — Infusion HALTED ***");
                Serial.println("[BTN]     Press button again to reset.");
            } else {
                infusion_active = true;
                alarm_flag      = false;
                digitalWrite(LED_ALARM,  LOW);
                digitalWrite(LED_ACTIVE, HIGH);
                Serial.println("[BTN] Operator reset — Infusion RESUMED. Alarm cleared.");
            }
        }

        btnTask_exec++;
        if ((int32_t)(xTaskGetTickCount() - deadline) > 0) {
            btnTask_miss++;
            Serial.println("[WARN] ButtonTask missed deadline!");
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

// ============================================================
// TASK 2 — Dose Calculation
// Period : 250 ms | WCET : 40 ms | Priority : 2
//
// Sawtooth dose ramp 0.5 → 1.5 mL over 11 cycles.
// Overdose alarm latches at > 1.40 mL — cleared only by operator.
// ============================================================
void taskDose(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(250);
    uint32_t cycle = 0;

    while (1) {
        TickType_t deadline = last_wake + period;
        TickType_t t_start  = xTaskGetTickCount();

        if (infusion_active) {
            cycle++;
            dose_volume_ml = 0.5f + (float)(cycle % 11) * 0.1f;
            do_work(40);  // Simulate 40 ms WCET

            if (dose_volume_ml > 1.40f && !alarm_flag) {
                alarm_flag = true;
                Serial.printf("[DOSE] *** OVERDOSE RISK: %.2f mL > 1.40 mL — ALARM LATCHED ***\r\n",
                              dose_volume_ml);
            }
            // FIX: single printf (original had this duplicated — removed)
            Serial.printf("[DOSE] Computed: %.2f mL%s\r\n",
                          dose_volume_ml, alarm_flag ? " [ALARM]" : "");
        } else {
            Serial.println("[DOSE] Halted — skipping calculation.");
        }

        doseTask_exec++;
        TickType_t t_end = xTaskGetTickCount();
        if ((int32_t)(t_end - deadline) > 0) {
            doseTask_miss++;
            Serial.printf("[WARN] DoseTask missed deadline! Exec≈%lu ms\r\n",
                          (uint32_t)((t_end - t_start) * portTICK_PERIOD_MS));
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

// ============================================================
// TASK 3 — Stepper Motor Control
// Period : 10 ms | WCET : ~2 ms | Priority : 4 (highest periodic, RMS)
//
// PROBLEM MODE (CONTENTION_DEMO=1):
//   Acquires i2cMutex every step with 8 ms timeout.
//   When LCD holds mutex for its full 100 ms WCET, Motor blocks
//   and misses up to 10 consecutive steps per LCD refresh — the
//   scheduling problem described in the spec.
//
// SOLUTION MODE (CONTENTION_DEMO=0):
//   Motor step uses LEDC PWM only — no I2C access per step.
//   The motor driver enable pin is updated via I2C ONLY when
//   infusion_active changes state (button press). State changes
//   are infrequent → contention eliminated. Zero missed steps
//   due to I2C blocking in steady state.
// ============================================================
void taskMotor(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);
    static bool step_state = false;

    while (1) {
        TickType_t deadline = last_wake + period;
        bool missed_this_cycle = false;

#if !CONTENTION_DEMO
        // SOLUTION: update motor enable pin only on state change (infrequent)
        bool cur_state = infusion_active;
        if (cur_state != infusion_prev_state) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(8)) == pdTRUE) {
                Serial.printf("[MOTOR] Enable pin → %s (state change)\r\n",
                              cur_state ? "ENABLED" : "DISABLED");
                infusion_prev_state = cur_state;
                xSemaphoreGive(i2cMutex);
            }
            // If mutex not available, retry next cycle — not time-critical
        }
#endif

        if (infusion_active) {
#if CONTENTION_DEMO
            // PROBLEM: acquire mutex every step → contention with LCD
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(8)) == pdTRUE) {
                step_state = !step_state;
                ledcWrite(1, step_state ? 128 : 0);
                xSemaphoreGive(i2cMutex);
            } else {
                // LCD holds mutex — missed step (the scheduling problem)
                motorTask_miss++;
                missed_this_cycle = true;
                Serial.printf("[MOTOR] Step MISSED — I2C contention (total: %lu)\r\n",
                              motorTask_miss);
            }
#else
            // SOLUTION: PWM only, no I2C needed per step
            step_state = !step_state;
            ledcWrite(1, step_state ? 128 : 0);
#endif
        } else {
            ledcWrite(1, 0);  // Disable motor when halted
        }

        motorTask_exec++;
        if (!missed_this_cycle && (int32_t)(xTaskGetTickCount() - deadline) > 0) {
            motorTask_miss++;
            Serial.println("[WARN] MotorTask missed deadline (non-mutex cause)!");
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

// ============================================================
// TASK 4 — LCD Display Update
// Period : 500 ms | WCET : 100 ms | Priority : 1 (lowest periodic)
//
// Holds i2cMutex for 100 ms WCET — root cause of motor step
// misses in CONTENTION_DEMO mode. Priority inheritance (built
// into xSemaphoreCreateMutex) boosts LCD to Motor priority while
// it holds the mutex, preventing lower-priority tasks from
// delaying it further, but Motor must still wait the full 100 ms.
// ============================================================
void taskLCD(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);

    while (1) {
        TickType_t deadline = last_wake + period;

        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
            lcd.clear();
            lcd.setCursor(0, 0);
            if (!infusion_active) {
                lcd.print("!! EMERGENCY !! ");
                lcd.setCursor(0, 1);
                lcd.print("PRESS BTN RESET ");
            } else if (alarm_flag) {
                lcd.print("!OVERDOSE ALERT!");
                lcd.setCursor(0, 1);
                lcd.print("HALT & PRESS BTN");
            } else {
                lcd.print("Dose:");
                lcd.setCursor(6, 0);
                lcd.print(dose_volume_ml, 2);
                lcd.setCursor(12, 0);
                lcd.print(" mL");
                lcd.setCursor(0, 1);
                lcd.print("Status: ACTIVE  ");
            }
            do_work(100);  // Simulate 100 ms I2C WCET — holds mutex
            xSemaphoreGive(i2cMutex);
        } else {
            lcdTask_skip++;
            Serial.printf("[LCD]  Mutex timeout — frame skipped (total: %lu)\r\n",
                          lcdTask_skip);
        }

        lcdTask_exec++;
        if ((int32_t)(xTaskGetTickCount() - deadline) > 0) {
            lcdTask_miss++;
            Serial.println("[WARN] LCDTask missed deadline!");
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

// ============================================================
// MONITOR TASK — Runtime stats every 5 s | Priority : 0 (IDLE)
// ============================================================
void taskMonitor(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5000);

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        monitorTask_exec++;

        Serial.println("\n============= RMS RUNTIME STATS =============");
#if CONTENTION_DEMO
        Serial.println("  MODE    : CONTENTION PROBLEM DEMO");
#else
        Serial.println("  MODE    : SOLUTION (decoupled motor enable)");
#endif
        Serial.println("  Task     | Exec  | Misses | StackFree(words)");
        Serial.println("  ---------|-------|--------|----------------");
        Serial.printf( "  Button   | %5lu | %6lu | %u\r\n",
                       btnTask_exec,   btnTask_miss,
                       (unsigned)uxTaskGetStackHighWaterMark(hButton));
        Serial.printf( "  Motor    | %5lu | %6lu | %u\r\n",
                       motorTask_exec, motorTask_miss,
                       (unsigned)uxTaskGetStackHighWaterMark(hMotor));
        Serial.printf( "  Dose     | %5lu | %6lu | %u\r\n",
                       doseTask_exec,  doseTask_miss,
                       (unsigned)uxTaskGetStackHighWaterMark(hDose));
        Serial.printf( "  LCD      | %5lu | %6lu | %u (skipped: %lu)\r\n",
                       lcdTask_exec,   lcdTask_miss,
                       (unsigned)uxTaskGetStackHighWaterMark(hLCD), lcdTask_skip);
        Serial.printf( "  Monitor  | %5lu |      - | %u\r\n",
                       monitorTask_exec,
                       (unsigned)uxTaskGetStackHighWaterMark(hMonitor));
        Serial.println("  -------------------------------------------");
        Serial.printf( "  Infusion : %s\r\n", infusion_active ? "ACTIVE"  : "HALTED");
        Serial.printf( "  Alarm    : %s\r\n", alarm_flag      ? "LATCHED" : "CLEAR");
        Serial.printf( "  Dose vol : %.2f mL\r\n", dose_volume_ml);
        Serial.printf( "  FreeHeap : %u bytes\r\n", (unsigned)xPortGetFreeHeapSize());
        Serial.println("=============================================\n");
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_ALARM,  OUTPUT);
    pinMode(LED_ACTIVE, OUTPUT);
    digitalWrite(LED_ALARM,  LOW);
    digitalWrite(LED_ACTIVE, HIGH);

    ledcSetup(1, 1000, 8);
    ledcAttachPin(MOTOR_PIN, 1);

    i2cMutex = xSemaphoreCreateMutex();
    configASSERT(i2cMutex != NULL);

    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0, 0); lcd.print("Infusion Pump");
    lcd.setCursor(0, 1); lcd.print("Booting ...");
    delay(1000);

    // ── Schedulability Analysis ──────────────────────────────
    Serial.println("\n====== Medical Infusion Pump — RMS Schedulability ======");
    Serial.println("Task   | Period(ms) | WCET(ms) | Util (Ci/Ti)");
    Serial.println("-------|------------|----------|-------------");
    Serial.println("Button |     20     |    <1    |   0.050");
    Serial.println("Motor  |     10     |     2    |   0.200");
    Serial.println("Dose   |    250     |    40    |   0.160");
    Serial.println("LCD    |    500     |   100    |   0.200");
    Serial.println("-------|------------|----------|-------------");
    Serial.println("Total U = 0.610");
    Serial.println("U_lub(n=4) = 4*(2^(1/4)-1) = 0.757");
    Serial.println("0.610 < 0.757  =>  SCHEDULABLE (utilisation test PASSES)");
    Serial.println();
    Serial.println("Response Time Analysis (worst-case, RMS priority order):");
    Serial.println("  R_Motor : R0=2, R1=2+ceil(2/20)*1 = 3 ms   < T=10ms  PASS");
    Serial.println("  R_Dose  : iterates 40->59 ms               < T=250ms PASS");
    Serial.println("  R_LCD   : iterates 100->260 ms             < T=500ms PASS");
    Serial.println("  (Full iteration table in Part 1 report)");
    Serial.println();
    Serial.println("KNOWN CONTENTION:");
    Serial.println("  i2cMutex held by LCD for 100 ms WCET.");
    Serial.println("  Motor (T=10ms) blocks -> up to 10 missed steps per LCD cycle.");
    Serial.println("  Priority inheritance active (xSemaphoreCreateMutex) but");
    Serial.println("  cannot reduce the 100 ms blocking window.");
    Serial.println();
#if CONTENTION_DEMO
    Serial.println("MODE: CONTENTION PROBLEM (CONTENTION_DEMO=1)");
    Serial.println("      Observe motorTask_miss increase in runtime stats.");
#else
    Serial.println("MODE: SOLUTION (CONTENTION_DEMO=0)");
    Serial.println("      Motor enable updated only on state change.");
    Serial.println("      Per-step I2C contention: ELIMINATED.");
    Serial.println("      Expected motorTask_miss = 0 in runtime stats.");
#endif
    Serial.println("All tasks pinned to Core 1 — single-core RMS model.");
    Serial.println("========================================================\n");

    // Create tasks — all pinned to Core 1 for deterministic RMS behaviour
    // Priority: Button=5(safety override), Motor=4(RMS-P1), Dose=2, LCD=1
    BaseType_t r;
    r = xTaskCreatePinnedToCore(taskButton,  "ButtonTask",  2048, NULL, 5, &hButton,  1);
    configASSERT(r == pdPASS);
    r = xTaskCreatePinnedToCore(taskMotor,   "MotorTask",   2048, NULL, 4, &hMotor,   1);
    configASSERT(r == pdPASS);
    r = xTaskCreatePinnedToCore(taskDose,    "DoseTask",    4096, NULL, 2, &hDose,    1);
    configASSERT(r == pdPASS);
    r = xTaskCreatePinnedToCore(taskLCD,     "LCDTask",     4096, NULL, 1, &hLCD,     1);
    configASSERT(r == pdPASS);
    r = xTaskCreatePinnedToCore(taskMonitor, "MonitorTask", 4096, NULL, 0, &hMonitor, 1);
    configASSERT(r == pdPASS);

    Serial.println("All tasks created on Core 1. Pump running.\n");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}