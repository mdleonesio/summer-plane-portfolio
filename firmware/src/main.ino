#include <Servo.h>
#include <IBusBM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>

File logFile;
const int chipSelect = BUILTIN_SDCARD;
bool isLogging = false;
bool lastSWDState = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
IBusBM iBus;

Servo servoPitch, servoRoll, servoYaw, throttle;

enum MissionPhase {
  IDLE,
  THROTTLE_RAMP,
  ELEVATOR_RAMP,
  CLIMB_CONTROL,
  CRUISE_CONTROL
};

MissionPhase currentPhase = IDLE;
unsigned long missionStartTime = 0;

const int pinPitch = 6;
const int pinRoll  = 7;
const int pinYaw   = 8;
const int pinThrottle = 9;

const int CH_ROLL     = 3;
const int CH_PITCH    = 1;
const int CH_THROTTLE = 2;
const int CH_YAW      = 0;
const int CH_SWA      = 4;  
const int CH_KNOB1    = 8;
const int CH_KNOB2    = 9;

float kP = 1.0, kD = .5;
float yawKp = 0.3, yawKd = 0.1;

float pitch = 0, pitchRate = 0;
float yaw = 0, yawRate = 0;

float pitchError = 0;
float yawError = 0;

float elevatorCmd = 90;
float rudderCmd = 90;

int rollOut = 90;
int pitchOut = 90;
int yawOut = 90;
int throttleOut = 1000;

float targetPitchClimb = 13;
float targetYaw = 0;
bool takeoffYawInit = false;

bool elevatorRampStarted = false;
bool elevatorFrozen = false;
unsigned long elevatorRampStartTime = 0;
unsigned long freezeStartTime = 0;
int frozenElevator = 90;

unsigned long throttleRampEndTime = 0;
unsigned long throttleRampDuration = 3000; 
unsigned long elevatorRampDuration = 3000;  
unsigned long climbStartTime = 0;  
unsigned long climbDuration = 10000;        

unsigned long pitchTime = 0;

void setup() {

  if (!SD.begin(chipSelect)) {
    Serial.println("SD init failed!");
  } else {
    Serial.println("SD card ready.");
  }

  Serial.begin(9600);
  Wire.begin();

  bno.begin();
  bno.setExtCrystalUse(true);
  iBus.begin(Serial1);

  servoPitch.attach(pinPitch);
  servoRoll.attach(pinRoll);
  servoYaw.attach(pinYaw);
  throttle.attach(pinThrottle);

  throttle.writeMicroseconds(1000);
  delay(3000);

  Serial.println("Setup complete");

}

void loop() {
  updateMissionState(); 
  runMission();          
  writeOutputs();  
  checkLoggingSwitch();    
  logFlightData();    

 //Serial.print(" throttle | "); Serial.println(throttleOut);
}


void updateMissionState() {
  static bool lastSwitchState = false;
  bool switchArmed = (iBus.readChannel(CH_SWA) > 1800);

  if (switchArmed && !lastSwitchState) {
    
    resetMissionState();

    missionStartTime = millis();
    currentPhase = THROTTLE_RAMP;

    int rawKnob1 = iBus.readChannel(CH_KNOB1);
    throttleRampDuration = map(rawKnob1, 1000, 2000, 1000, 10000);
    throttleRampEndTime = throttleRampDuration;

    Serial.println(" MISSION STARTED");
  }

  if (!switchArmed && lastSwitchState) {
    currentPhase = IDLE;
    Serial.println(" MISSION STOPPED");
  }

  lastSwitchState = switchArmed;
}

void runMission() {
  unsigned long t = millis() - missionStartTime;

  switch (currentPhase) {
    case IDLE:
      runManualMode();
      break;

    case THROTTLE_RAMP:
      runThrottleRamp(t);
      if (t > throttleRampEndTime) {
        currentPhase = ELEVATOR_RAMP;
      }
      break;

    case ELEVATOR_RAMP:
      runElevatorRamp(t);
      break;

    case CLIMB_CONTROL:
      runClimbControl();
      if (millis() - climbStartTime > climbDuration) {
        currentPhase = CRUISE_CONTROL;
      }
      break;

    case CRUISE_CONTROL:
      runCruiseControl();
      break;
  }
}

void runManualMode() {
  rollOut     = iBus.readChannel(CH_ROLL);
  pitchOut    = iBus.readChannel(CH_PITCH);
  throttleOut = iBus.readChannel(CH_THROTTLE);
  yawOut      = iBus.readChannel(CH_YAW);

  servoPitch.write(pitchOut);
  servoRoll.write(rollOut);
  servoYaw.write(yawOut);
  throttle.writeMicroseconds(throttleOut);
}



void runThrottleRamp(unsigned long t) {

  if (t < throttleRampDuration) {
    throttleOut = map(t, 0, throttleRampDuration, 1300, 1800);
  } else {
    throttleOut = 1800;
  }


  if (throttleOut > 1500) {
    if (!takeoffYawInit) {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      targetYaw = euler.x();
      takeoffYawInit = true;
    }
    
    yawOut = iBus.readChannel(CH_YAW);
  } else {
    yawOut = 90; 
  }


  Serial.print("ThrottleRamp | t: "); Serial.print(t);
  Serial.print(" | throttleOut: "); Serial.print(throttleOut);
  Serial.print(" | rudder: "); Serial.println(yawOut);
}

//void runRudderControl() {
//  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//
//  if (!takeoffYawInit) {
//  targetYaw = euler.x(); 
//  takeoffYawInit = true;
//  }
//  
//  float currentYaw = euler.x();
//  float yawRate = gyro.z();
//
//  yawError = targetYaw - currentYaw;
//  if (yawError > 180) yawError -= 360;
//  if (yawError < -180) yawError += 360;
//
//  if (abs(yawError) < 0.5) yawError = 0;
//  if (abs(yawRate) < 0.1) yawRate = 0;
//
//  rudderCmd = 90 - yawKp * yawError - yawKd * yawRate;
//  yawOut = constrain(rudderCmd, 40, 140);
//
//}


void runElevatorRamp(unsigned long t) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float currentPitch = -euler.z(); 

  int neutralElevator = 90;
  int maxElevator = 60;
  int elevatorRange = abs(maxElevator - neutralElevator);

  yawOut = 90;

  int rawKnob = iBus.readChannel(CH_KNOB2);
  unsigned long elevatorRampDuration = map(rawKnob, 1000, 2000, 1000, 10000); 

  if (throttleOut > 1600 && !elevatorRampStarted) {
    elevatorRampStartTime = millis();
    elevatorRampStarted = true;
    Serial.println("Elevator ramp started");
  }


  if (!elevatorFrozen && elevatorRampStarted) {
    unsigned long rampElapsed = millis() - elevatorRampStartTime;

    if (rampElapsed < elevatorRampDuration) {
      pitchOut = neutralElevator - (elevatorRange * rampElapsed) / elevatorRampDuration;
    } else {
      pitchOut = maxElevator;
    }


    if (currentPitch > 8.0) {
      frozenElevator = pitchOut;
      freezeStartTime = millis();
      elevatorFrozen = true;
      pitchTime = millis();
      Serial.println("Elevator frozen for transition wait");
    }
  }


  if (elevatorFrozen) {
    pitchOut = frozenElevator;

    if (millis() - freezeStartTime > 1500) {
      currentPhase = CLIMB_CONTROL;
      climbStartTime = millis(); 
      climbDuration = map(iBus.readChannel(CH_KNOB2), 1000, 2000, 3000, 10000);
      elevatorRampStarted = false;
      elevatorFrozen = false;
      Serial.println("→ CLIMB CONTROL");
    }
  }


  Serial.print("ElevatorRamp | Pitch: "); Serial.print(currentPitch);
  Serial.print(" | ElevatorRampDuration: "); Serial.print(throttleOut);
  Serial.print(" | Elevator: "); Serial.println(pitchOut);
}



void runClimbControl() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  pitch = -euler.z(); 
  pitchRate = gyro.x();
  yaw = euler.x();
  yawRate = gyro.z();

  pitchError = targetPitchClimb - pitch;
  float elevatorCmd = kP * pitchError - kD * pitchRate;
  pitchOut = constrain(90 - elevatorCmd, 60, 120);  

  
  if (!takeoffYawInit) {
    targetYaw = yaw;
    takeoffYawInit = true;
  }

  yawError = targetYaw - yaw;
  if (yawError > 180) yawError -= 360;
  if (yawError < -180) yawError += 360;

  float rudderCmd = 90 - yawKp * yawError - yawKd * yawRate;
  yawOut = constrain(rudderCmd, 40, 140);

  throttleOut = 1800;

  Serial.print("CLIMB | Pitch: "); Serial.print(pitch);
  Serial.print(" | PitchOut: "); Serial.print(pitchOut);
  Serial.print(" | Yaw: "); Serial.print(yaw);
  Serial.print(" | YawOut: "); Serial.println(yawOut);
}


void runCruiseControl() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float pitch = -euler.z();
  float pitchRate = gyro.x();
  float yaw = euler.x();
  float yawRate = gyro.z();

  float targetPitchCruise = 0; 
  float pitchError = targetPitchCruise - pitch;
  float elevatorCmd = kP * pitchError - kD * pitchRate;
  pitchOut = constrain(90 - elevatorCmd, 60, 120);

  float yawError = targetYaw - yaw;
  if (yawError > 180) yawError -= 360;
  if (yawError < -180) yawError += 360;

  float rudderCmd = 90 - yawKp * yawError - yawKd * yawRate;
  yawOut = constrain(rudderCmd, 40, 140);

  throttleOut = 1500; 

  Serial.println("CRUISE |");
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" | Elevator: "); Serial.print(pitchOut);
  Serial.print(" | Yaw: "); Serial.print(yaw);
  Serial.print(" | Rudder: "); Serial.print(yawOut);
  Serial.print(" | Throttle: "); Serial.println(throttleOut);
}


void writeOutputs() {
  servoPitch.write(pitchOut);
  servoRoll.write(rollOut);
  servoYaw.write(yawOut);
  throttle.writeMicroseconds(throttleOut);
}

void checkLoggingSwitch() {
  int swd = iBus.readChannel(7);
  bool swdDown = swd > 1800;

  if (swdDown && !lastSWDState) {
    logFile = SD.open("log.csv", FILE_WRITE);
    if (logFile) {
      Serial.println("✅ Logging started.");
      logFile.println("Time_ms,Phase,Pitch,PitchRate,Elevator,Yaw,YawRate,Rudder,Throttle");
      isLogging = true;
    } else {
      Serial.println("❌ Failed to open log.csv");
      isLogging = false;
    }
  }

  if (!swdDown && lastSWDState && isLogging) {
    logFile.println("LOG STOPPED");
    logFile.close();
    isLogging = false;
    Serial.println("Logging stopped.");
  }

  lastSWDState = swdDown;
}

void resetMissionState() {
  elevatorRampStarted = false;
  elevatorFrozen = false;
  elevatorRampStartTime = 0;
  freezeStartTime = 0;
  frozenElevator = 90;

  throttleRampEndTime = 0;
  throttleRampDuration = 0;

  pitchTime = 0;
  climbStartTime = 0;
  climbDuration = 0;

  takeoffYawInit = false;
}


void logFlightData() {
  static unsigned long lastLogTime = 0;
  unsigned long now = millis();

  if (!isLogging || now - lastLogTime < 50) return;  

  lastLogTime = now;

  String phaseName;
  switch (currentPhase) {
    case IDLE:            phaseName = "IDLE"; break;
    case THROTTLE_RAMP:   phaseName = "THROTTLE_RAMP"; break;
    case ELEVATOR_RAMP:   phaseName = "ELEVATOR_RAMP"; break;
    case CLIMB_CONTROL:   phaseName = "CLIMB_CONTROL"; break;
    case CRUISE_CONTROL:  phaseName = "CRUISE_CONTROL"; break;
  }

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float logPitch = -euler.z();
  float logPitchRate = gyro.x();
  float logYaw = euler.x();
  float logYawRate = gyro.z();

  logFile.print(now);
  logFile.print(",");
  logFile.print(phaseName);
  logFile.print(",");
  logFile.print(logPitch);
  logFile.print(",");
  logFile.print(logPitchRate);
  logFile.print(",");
  logFile.print(pitchOut);
  logFile.print(",");
  logFile.print(logYaw);
  logFile.print(",");
  logFile.print(logYawRate);
  logFile.print(",");
  logFile.print(yawOut);
  logFile.print(",");
  logFile.println(throttleOut);

  logFile.flush(); 
}
