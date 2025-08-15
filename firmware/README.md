# Firmware/Hardware

Transmitter/Reciever: **FS-i6X**
Board: **Teensy 4.1**  
IMU: **Adafruit BNO055** (Adafruit Unified Sensor + BNO055 libs)  
RC: **IBusBM** on `Serial1`
Logging: **Micro SD card** → `log.csv`  

## Build (Arduino IDE)
1. File → Open → `firmware/src/main.ino`
2. Tools → Board: **Teensy 4.1**; Port: your Teensy.
3. Install libs: **Servo**, **IBusBM**, **Adafruit Unified Sensor**, **Adafruit BNO055**, **SD**.
5. Serial Monitor 9600 for debug. 

## Channels (as expected in code)
- CH_YAW (0), CH_PITCH (1), CH_THROTTLE (2), CH_ROLL (3)
- CH_SWA (4): mission start/stop (AUTO toggle)
- CH_SWD (7): SD logging start/stop
- CH_KNOB1 (8): throttle ramp time (1–10 s mapped)
- CH_KNOB2 (9): elevator ramp time / climb duration

## Phases
`IDLE → THROTTLE_RAMP → ELEVATOR_RAMP → CLIMB_CONTROL → CRUISE_CONTROL`  
Manual override at any time by switching out of AUTO.

## Logging columns
`Time_ms, Phase, Pitch, PitchRate, Elevator, Yaw, YawRate, Rudder, Throttle`
