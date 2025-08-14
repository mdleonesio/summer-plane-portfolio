# Autonomous RC Airplane — Scratch-Built (Solo)

**Summary:** From a blank page to a working stabilization testbed: I designed and built an autonomous RC airplane from scratch—airframe, avionics, and control software—and iterated through bench, taxi, and flight tests to validate core stabilization and create a reusable platform for future autonomy.

**Goal** — Build a reusable RC platform for autonomy (stabilization today; cruise/waypoints next).

**Approach** — Simple foamboard airframe; onboard control architecture (microcontroller + IMU → control laws → RC override → servos/ESC) with on-board logging; staged testing (bench → taxi → flight); fast build–measure–learn cycles.

**Results** — Achieved liftoff and short controlled segments; validated stabilization; established a maintainable platform for systematic tuning and future iterations.

**What’s next** — Integrate airspeed sensing, refine gains/gain scheduling, add cruise hold, and expand logging/analysis for flight tuning.

## Quick Facts

| Area | Summary |
|---|---|
| Timeframe | May–Aug 2025 • Solo |
| Airframe | Foamboard, constant-chord wing; tricycle gear; ~30″ fuselage |
| Objective | Stabilized flight testbed for future autonomy |
| Control Architecture | MCU + IMU → control laws (PD/PID, state machine) → RC override → servos/ESC |
| Data | On-board logging (attitude + rates); MATLAB/Simulink SITL checks |
| Test Flow | Bench → Taxi → Flight; achieved liftoff + short controlled segments |
| Toolchain | SolidWorks, Excel, Arduino IDE/C++, XFLR5/XFOIL, MATLAB/Simulink |

## Architecture

```mermaid
flowchart LR
  subgraph Onboard_System
    IMU["IMU<br/>(attitude & rates)"]
    RC["RC Receiver<br/>(sticks, switches, knobs)"]
    Control["Control Laws<br/>(PD/PID + State Machine)"]
    Mixer["Command Mixer<br/>(override + trims + limits)"]
    Servos["Servos<br/>(elevator / rudder / aileron)"]
    ESC["ESC / Throttle"]
    Logger["Logger"]
    SD["SD Card"]
    Health["Arming & Failsafe"]
  end

  IMU -->|sensor data| Control
  RC -->|channels| Health
  Health --> Control
  Control --> Mixer
  RC -->|override| Mixer
  Mixer --> Servos
  Mixer --> ESC

  %% Logging
  IMU --> Logger
  Control --> Logger
  Logger --> SD

  %% Offboard sources (dashed)
  Sim["MATLAB/Simulink & XFLR5"] -. "models/params" .-> Control
  Tuning["Ground Tuning<br/>(knobs/switches)"] --> RC

