#include <Romi32U4.h>
#include <math.h>
#include "Adafruit_BNO08x_RVC.h"


//-Robot Info-
const double wheelCircumference = M_PI * 0.07;
const int ticks = 1440;
const double mPerTick = wheelCircumference / ticks;
//double trackWidth = 0.13;


//-Movement Consts-
const int maxPower = 300;
int maxTurnPower = 100;
const int minSpeed[3] = { 30, 50, 75 };
const float accelSpeed[3] = { 0.2, 0.15, 0.1 };
const float decelSpeed[3] = { 0.2, 0.2, 0.2 };


//-Movement Vars-
double xTarget = 0;
double yTarget = 1;
double desiredTheta = 0;
double maxRotationalVelocity = 120.0;
double startTurn = 0.141;
int turnForwardSpeed = 50;


//-Pose Vars-
double theta = 0.0;
double xPos = 0.0;
double yPos = 0.0;
double velocity = 0.0;
double rotationalVelocity = 0.0;
double dTheta = 0;
double distanceToTarget = 0;


//-Commands-
double turnCommand = 0.0;
double forwardCommand = 0.0;
double totalCommandLeft = 0.0;
double totalCommandRight = 0.0;


//-Pose Calculation Volatiles-
//Dist
double gyroInit = 0.0;
//Timer
unsigned long currentTime = 0;
unsigned long posePrevMicros = 0, turnPrevMicros = 0, fwdPrevMicros = 0;


//-Misc Logic-
char bottleCount = 0;
int count = 0;
bool justTurned = false;


//-Object Definitions-
Romi32U4Motors motors;
Romi32U4Encoders encoders;
Romi32U4ButtonA buttonA;
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
BNO08x_RVC_Data heading;


void updatePose() {
  theta = heading.yaw - gyroInit;
  static double thetaLast = 0;
  static long encLeftLast = 0;
  static long encRightLast = 0;
  long encLeft = encoders.getCountsLeft();
  long encRight = encoders.getCountsRight();
  int leftTravel = encLeft - encLeftLast;
  int rightTravel = encRight - encRightLast;
  int netTravel = (leftTravel + rightTravel) / 2;
  xPos = xPos + mPerTick * (netTravel)*sin(theta * M_PI / 180);
  yPos = yPos + mPerTick * (netTravel)*cos(theta * M_PI / 180);
  encLeftLast = encLeft;
  encRightLast = encRight;
  currentTime = micros();
  unsigned long timeElapsed = currentTime - posePrevMicros;
  posePrevMicros = currentTime;
  velocity = mPerTick * netTravel * 1000000.0 / timeElapsed;
  rotationalVelocity = (theta - thetaLast) * 1000000.0 / timeElapsed;
  thetaLast = theta;
}


void calculateNextTarget() {
  desiredTheta = atan2((xTarget - xPos), (yTarget - yPos)) * 180 / M_PI;
  dTheta = desiredTheta - theta;
  if (dTheta > 180) {
    dTheta -= 360;
  } else if (dTheta < -180) {
    dTheta += 360;
  }
  distanceToTarget = sqrt(pow(xTarget - xPos, 2) + pow(yTarget - yPos, 2));
}


void turnPID() {
  const double Kp = 7.5;
  const double Kd = 0.03;
  static double lastError = 0;
  static double lastTurnCommand = 0;
  currentTime = micros();
  double dt = (currentTime - turnPrevMicros) / 1000000.0;  // convert µs to seconds
  if (dt <= 0) dt = 1e-6;


  double error = dTheta;  // heading error in degrees
  double derivative = (error - lastError) / dt;
  if (justTurned) {
    derivative = 0;
    justTurned = false;
  }

  turnCommand = Kp * error + Kd * derivative;
  /*if (fabs(rotationalVelocity) > maxRotationalVelocity) {
    turnCommand = turnCommand * maxRotationalVelocity / fabs(rotationalVelocity);
  }*/
  lastError = error;
  turnPrevMicros = currentTime;
  if (dTheta >= 45) {
    if (fabs(lastTurnCommand - turnCommand) >= 15) {
      if (turnCommand > lastTurnCommand) {
        turnCommand = lastTurnCommand + 15;
      } else {
        turnCommand = lastTurnCommand - 15;
      }
    }
  }
  lastTurnCommand = turnCommand;
  turnCommand = constrain(turnCommand, -maxTurnPower, maxTurnPower);
}


/*void forwardPID() {
  const double fKp = 3000;
  const double fKd = 2.7;
  static double lastDistError = 0;
  static double lastForwardCommand = 0;
  currentTime = micros();
  double dt = (currentTime - fwdPrevMicros) / 1000000.0;  // convert µs to seconds
  if (dt <= 0) dt = 1e-6;  
  double distError = distanceToTarget;
  double forwardDerivative = (distError - lastDistError) / dt;
  forwardCommand = fKp * distanceToTarget + fKd * forwardDerivative;
  if (fabs(lastForwardCommand-forwardCommand) >= 15){
    if (forwardCommand > lastForwardCommand){
      forwardCommand = lastForwardCommand + 15;
    } else {
      forwardCommand = lastForwardCommand -15;
    }
  }
  forwardCommand = constrain(forwardCommand, 0, maxPower - fabs(turnCommand));
  lastDistError = distError;
  lastForwardCommand = forwardCommand;
  fwdPrevMicros = currentTime;
}*/

void forwardPID() {
  // --- Tunables for distance-based profile ---
  const double DECEL_START = 0.35;  // m: begin ramping down
  const double DECEL_END = 0.15;    // m: finish ramp (min speed here)
  const double VEL_HIGH = 300.0;    // motor units at >= DECEL_START
  const double VEL_LOW = 150.0;     // motor units at <= DECEL_END

  // --- PD gains (use your current values or tune) ---
  const double fKp = 3000.0;
  const double fKd = 2.7;

  // --- Timing / derivative ---
  static double lastDistError = 0.0;
  static double lastForwardCommand = 0.0;
  currentTime = micros();
  double dt = (currentTime - fwdPrevMicros) / 1e6;
  if (dt <= 0) dt = 1e-6;

  // --- PD on distance-to-target ---
  double distError = distanceToTarget;              // m
  double dDist = (distError - lastDistError) / dt;  // m/s
  double rawFwd = fKp * distError + fKd * dDist;    // motor units (unclamped)
  if (rawFwd < 0) rawFwd = 0;                       // no reverse here

  // --- Distance-based max forward (300 -> 100 over 0.35m -> 0.15m) ---
  double maxByDist;
  if (distError >= DECEL_START) {
    maxByDist = VEL_HIGH;
  } else if (distError <= DECEL_END) {
    maxByDist = VEL_LOW;
  } else {
    double t = (DECEL_START - distError) / (DECEL_START - DECEL_END);  // 0..1
    maxByDist = VEL_HIGH + (VEL_LOW - VEL_HIGH) * t;                   // lerp
  }

  // --- Combine with motor budget (optional safety) ---
  // Keep if you still want to reserve room for turning:
  double budgetCap = maxPower - fabs(turnCommand);
  if (budgetCap < 0) budgetCap = 0;

  // Final cap = min(distance profile, budget)
  double finalCap = maxByDist;
  if (budgetCap < finalCap) finalCap = budgetCap;

  // --- Output command ---
  forwardCommand = rawFwd;
  if (forwardCommand > finalCap) forwardCommand = finalCap;
  if (fabs(lastForwardCommand - forwardCommand) >= 15) {
    if (forwardCommand > lastForwardCommand) {
      forwardCommand = lastForwardCommand + 15;
    } else {
      forwardCommand = lastForwardCommand - 15;
    }
  }
  // --- Bookkeeping ---
  lastDistError = distError;
  fwdPrevMicros = currentTime;
  lastForwardCommand = forwardCommand;
}


void doTheThing() {
  turnPID();
  forwardPID();
  static int totalCommandLeftLast = 0;
  static int totalCommandRightLast = 0;
  totalCommandLeft = forwardCommand + turnCommand;
  totalCommandRight = forwardCommand - turnCommand;
  /*if (fabs(totalCommandLeftLast-totalCommandLeft) >= 15){
    if (totalCommandLeft > totalCommandLeftLast){
      totalCommandLeft = totalCommandLeftLast + 15;
    } else {
      totalCommandLeft = totalCommandLeftLast -15;
    }
  }
  if (fabs(totalCommandRightLast-totalCommandRight) >= 15){
    if (totalCommandRight > totalCommandRightLast){
      totalCommandRight = totalCommandRightLast + 15;
    } else {
      totalCommandRight = totalCommandRightLast -15;
    }
  }*/
  totalCommandLeftLast = totalCommandLeft;
  totalCommandRightLast = totalCommandRight;
  motors.setSpeeds(totalCommandLeft, totalCommandRight);
}






/*void executeTracking() {
  const double Kp = 7.5;
  const double fKp = 3000;
  const double Kd = 0.01;
  const double fKd = 2.7;
  static double lastError = 0;
  static double lastDistError = 0;
  unsigned long currentMicros = micros();
  double dt = (currentMicros - prevTime) / 1000000.0;  // convert µs to seconds
  if (dt <= 0) dt = 1e-6;                              // prevent division by zero


  double error = dTheta;  // heading error in degrees
  double derivative = (error - lastError) / dt;
  double distError = distanceToTarget;
  double forwardDerivative = (distError - lastDistError) / dt;


  double turnCommand = Kp * error + Kd * derivative;  // positive = turn right
  double forwardCommand = fKp * distanceToTarget + fKd * forwardDerivative;
  lastError = error;
  lastDistError = distError;
  prevTime = currentMicros;
  forwardCommand = constrain(forwardCommand, 0, 150);
  int leftMotorSpeed = constrain(forwardCommand + turnCommand, -maxPower, maxPower);
  int rightMotorSpeed = constrain(forwardCommand - turnCommand, -maxPower, maxPower);
  motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
}*/


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(50);
  motors.setSpeeds(0, 0);
  if (!rvc.begin(&Serial1)) {  // pass reference to Serial1
    Serial.println("Could not find BNO08x (check wiring, P0 jumper).");
    while (1) delay(10);
  }
  Serial.println("BNO08x found!");
  while (!buttonA.isPressed()) {
    rvc.read(&heading);
    //delay(1);
  }
  gyroInit = heading.yaw;
  posePrevMicros = turnPrevMicros = fwdPrevMicros = micros();
}
void loop() {
  //BNO08x_RVC_Data heading;
  if (rvc.read(&heading)) {
    static bool pt2 = false;
    static bool pt3 = false;
    updatePose();
    calculateNextTarget();
    //executeTracking();
    doTheThing();
    if (count % 10 == 0) {
      static long lastTimer;
      Serial.print("X: ");
      Serial.println(xPos, 4);
      Serial.print("Y: ");
      Serial.println(yPos, 4);
      Serial.print("Theta: ");
      Serial.println(theta);
      Serial.print("Velo: ");
      Serial.println(velocity);
      Serial.print("Desired Theta: ");
      Serial.println(desiredTheta);
      Serial.print("Distance to Target: ");
      Serial.println(distanceToTarget);
      Serial.print("dTheta: ");
      Serial.println(dTheta);
      Serial.print("Time per loop: ");
      Serial.println((micros() - lastTimer) / 10);
      lastTimer = micros();
    }
    count++;
    if (distanceToTarget <= startTurn && !pt3 && pt2) {
      pt3 = true;
      xTarget = .5;
      yTarget = .0;
      justTurned = true;
    }
    if (distanceToTarget <= startTurn && !pt2) {
      pt2 = true;
      xTarget = .5;
      yTarget = 1;
      justTurned = true;
    }
  }
}