#include <Romi32U4.h>
#include <math.h>
#include "Adafruit_BNO08x_RVC.h"
#include <PID_v1.h>

//-Robot Info-
const double wheelCircumference = M_PI * 0.07;
const int ticks = 1440;
const double mPerTick = wheelCircumference / ticks;
//double trackWidth = 0.141;

//-Movement Consts-
const int maxPower = 300;
int maxTurnPower = 100;
const int minPower[] = { 30, 50, 75 };
const float accelConst[] = { 2, 1.75, 1.5 };
const float decelConst[] = { 2, 2, 2 };
const float turnAccelConst[] = { 2880, 2880, 2880 };
const float turnDecelConst[] = { 2880, 2880, 2880 };
const float velocityConst[] = { 1, 1, 1 };
const float turnVelocityConst[] = { 720, 720, 720 };
double startTurn = 0.141;
int turnForwardSpeed = 50;
double maxRotationVelocity = 180.0;

//-Movement Vars-
double xTarget[] = { 0, -0.5, -0.5, 0 };
double yTarget[] = { 1, 1, 0, 0 };
double endPower[] = { 150, 150, 150, 150 };
int moveIndex = 0;

//-Pose Vars-
double theta = 0.0;
double xPos = 0.0;
double yPos = 0.0;
double velocity = 0.0;
double rotationVelocity = 0.0;
double dTheta = 0;
double distanceToTarget = 0;
double desiredTheta = 0;
double acceleration = 0;
double rotationAcceleration = 0;

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

//Wrapper Simplification
static inline double wrap180(double a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

//Decel Logic
static inline double accelHandle(double startVelo, double endVelo, double startPoint, double endPoint, double currentPos) {
  if (fabs(endPoint - startPoint) < 1e-9) return endVelo;
  double t = (currentPos - startPoint) / (endPoint - startPoint);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  return startVelo + (endVelo - startVelo) * t;
}

static inline double brakingDistance(double currentV, double targetV, double maxDecel) {
  if (maxDecel <= 0) return 0;  // avoid division by zero

  double v2 = currentV * currentV;
  double vt2 = targetV * targetV;

  // using s = (u^2 - v^2) / (2 * a)  with positive a for decel
  return (v2 - vt2) / (2.0 * maxDecel);
}

void updatePose() {
  theta = heading.yaw - gyroInit;
  static double thetaLast = 0;
  static long encLeftLast = 0;
  static long encRightLast = 0;
  static long microsLast = 0;
  static double rotationVelocityLast = 0;
  static double velocityLast = 0;
  long encLeft = encoders.getCountsLeft();
  long encRight = encoders.getCountsRight();
  int leftTravel = encLeft - encLeftLast;
  int rightTravel = encRight - encRightLast;
  int netTravel = (leftTravel + rightTravel) / 2;
  xPos = xPos + mPerTick * (netTravel)*sin(theta * M_PI / 180);
  yPos = yPos + mPerTick * (netTravel)*cos(theta * M_PI / 180);
  currentTime = micros();
  unsigned long timeElapsed = currentTime - microsLast;
  velocity = mPerTick * netTravel * 1000000.0 / timeElapsed;
  rotationVelocity = (theta - thetaLast) * 1000000.0 / timeElapsed;
  acceleration = (velocity - velocityLast) * 1000000.0 / timeElapsed;
  rotationAcceleration = (rotationVelocity - rotationVelocityLast) * 1000000.0 / timeElapsed;
  thetaLast = theta;
  microsLast = currentTime;
  encLeftLast = encLeft;
  encRightLast = encRight;
}

void calculateNextTarget() {
  desiredTheta = atan2((xTarget[moveIndex] - xPos), (yTarget[moveIndex] - yPos)) * 180 / M_PI;
  dTheta = wrap180(desiredTheta - theta);
  distanceToTarget = sqrt(pow(xTarget[moveIndex] - xPos, 2) + pow(yTarget[moveIndex] - yPos, 2));
}

void turnPID(double Kp = 7.5, double Ki = 0.0, double Kd = 0.03) {
  static double lastTheta = 0;
  static double lastTurnCommand = 0;
  static double
    currentTime = micros();
  double dt = (currentTime - turnPrevMicros) / 1000000.0;  // convert µs to seconds
  if (dt <= 0) dt = 1e-6;
  double error = dTheta;  // heading error in degrees
  double derivative = (theta - lastTheta) / dt;
  if (justTurned) {
    derivative = 0;
    justTurned = false;
  }
  turnCommand = Kp * error + Kd * derivative;
  /*if (fabs(rotationVelocity) > maxRotationVelocity) {
    turnCommand = turnCommand * maxRotationVelocity / fabs(rotationVelocity);
  }*/
  lastTheta = theta;
  turnPrevMicros = currentTime;
  if (fabs(dTheta) >= 45) {
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

void forwardPID() {
  // --- Tunables for distance-based profile ---
  const double DECEL_START = 0.35;             // m: begin ramping down
  const double DECEL_END = 0.15;               // m: finish ramp (min speed here)
  const double VEL_HIGH = 300.0;               // motor units at >= DECEL_START
  const double VEL_LOW = endPower[moveIndex];  // motor units at <= DECEL_END
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

void callLoop() {
  if (rvc.read(&heading)) {
    //Update Odom & Target
    updatePose();
    calculateNextTarget();
    //Prints
    printStuff();
  }
}

void pivot(float deg = 180.0) {
  desiredTheta = wrap180(theta + deg);

  double desiredVelocity = 0.0;         // deg/s, signed
  bool decelerating = false;
  double startVeloMag = 0.0;            // deg/s, magnitude at start of decel
  double startErrMag  = 0.0;            // deg, remaining angle at start of decel

  PID pivotPID(&rotationVelocity, &turnCommand, &desiredVelocity, 300, 0, 1, DIRECT);
  pivotPID.SetOutputLimits(-300, 300);
  pivotPID.SetSampleTime(5);
  pivotPID.SetMode(AUTOMATIC);

  long lastMicros = micros();

  while (true) {
    if (rvc.read(&heading)) {
      updatePose();

      // 1) Always recompute angle error from current pose
      dTheta = wrap180(desiredTheta - theta); // deg
      double errMag = fabs(dTheta);
      double dir    = (dTheta >= 0.0) ? 1.0 : -1.0;

      // dt
      long now = micros();
      double dt = (now - lastMicros) / 1e6;
      if (dt <= 0) dt = 1e-6;

      // 2) Accel phase with correct integration
      if (!decelerating) {
        desiredVelocity += dir * (turnAccelConst[bottleCount] * dt);   // deg/s^2 * s

        // 3) Proper clamp to Vmax
        double vmax = turnVelocityConst[bottleCount];
        if (fabs(desiredVelocity) > vmax) desiredVelocity = dir * vmax;

        // 5) Use magnitudes to decide when to brake
        if (brakingDistance(fabs(rotationVelocity), 0.0, turnDecelConst[bottleCount]) >= errMag) {
          decelerating = true;
          startVeloMag = fabs(rotationVelocity);
          startErrMag  = errMag;
        }
      } else {
        // 2+5) Distance-shaped decel with correct sign
        double vmag = accelHandle(startVeloMag, 0.0, startErrMag, 0.0, errMag);
        desiredVelocity = dir * vmag;
      }

      // 6) Drive the velocity PID
      pivotPID.Compute();
      motors.setSpeeds( turnCommand, -turnCommand );

      // Exit when angle is met and motion is nearly stopped
      if (errMag < 0.5 && fabs(rotationVelocity) < 1.0) {
        motors.setSpeeds(0, 0);
        break;
      }

      lastMicros = now;
      printStuff();
    }
  }
}

//void pivot(float deg = 180.0) {
  /*desiredTheta = wrap180(theta + deg);
  PID pivotPID(&theta, &turnCommand, &desiredTheta, 3, .3, .2, DIRECT);
  pivotPID.SetOutputLimits(-100, 100);
  pivotPID.SetSampleTime(5);
  pivotPID.SetMode(AUTOMATIC);
  while (true) {
    if (rvc.read(&heading)) {
      updatePose();
      dTheta = wrap180(desiredTheta - theta);
      pivotPID.Compute();
      motors.setSpeeds(turnCommand, -turnCommand);
      if (fabs(dTheta) < 0.5 && fabs(rotationVelocity) < 1) {
        motors.setSpeeds(0, 0);
        break;
      }
      printStuff();
    }
  }*/
  /*desiredTheta = wrap180(theta + deg);
  while (true) {
    if (rvc.read(&heading)) {
      updatePose();
      dTheta = wrap180(desiredTheta - theta);
      double targetVelo;
      if (dTheta > deg/2){
        targetVelo = accelHandle(100, 720, 0, deg/2, deg - dTheta);
      } else {
        targetVelo = accelHandle(720, 50, deg/2, deg, deg - dTheta);
      }
      double error = targetVelo - rotationVelocity;
      double kP = .5;
      double command = constrain(error*kP, -300, 300);
      motors.setSpeeds(command, -command);
      if (fabs(dTheta) < 0.5 && fabs(rotationVelocity) < 1) {
        motors.setSpeeds(0, 0);
        break;
      }
      printStuff();
    }
  }*/
  /*desiredTheta = wrap180(theta + deg);

  // optional: reset any filters/guards you use
  // justTurned = true; // if you gate derivatives elsewhere

  while (true) {
    if (rvc.read(&heading)) {
      updatePose();

      // Heading error, wrapped
      dTheta = wrap180(desiredTheta - theta);

      // Build target angular velocity profile (deg/s)
      // Symmetric accel/decel example using your accelHandle():
      double half = fabs(deg) * 0.5;
      double prog = fabs(deg) - fabs(dTheta);  // how far through the turn you are (deg)
      double targetVelo;
      if (prog < half) {
        // accelerate from 0 -> peak (you can use your turnVelocityConst[mode] as peak)
        targetVelo = accelHandle(0, turnVelocityConst[0], 0, half, prog);
      } else {
        // decelerate from peak -> 0
        targetVelo = accelHandle(turnVelocityConst[0], 0, half, fabs(deg), prog);
      }

      // Keep the sign consistent with turn direction
      targetVelo = copysign(targetVelo, deg);

      // Feed the velocity PID
      velSet = targetVelo;       // deg/s
      velIn = rotationVelocity;  // your measured deg/s
      velPID.Compute();

      // Apply differential motor command from PID output
      // Positive velOut should increase rotationVelocity toward velSet
      motors.setSpeeds(-velOut, velOut);

      // Exit condition: angle is met and we’re nearly stopped
      if (fabs(dTheta) < 0.5 && fabs(rotationVelocity) < 1.0) {
        motors.setSpeeds(0, 0);
        break;
      }
    }
  }*/
  /*desiredTheta = wrap180(theta + deg);
  double desiredVelocity = 0;
  bool accelerating = true;
  bool decelerating = false;
  double startVelo = 0;
  double startAng = 0;
  PID pivotPID(&rotationVelocity, &turnCommand, &desiredVelocity, 100, 0, 1, DIRECT);
  pivotPID.SetOutputLimits(-300, 300);
  pivotPID.SetSampleTime(5);
  pivotPID.SetMode(AUTOMATIC);

  long lastMicros = micros();

  while (true) {
    if (rvc.read(&heading)) {
      updatePose();
      dTheta = wrap180(desiredTheta - theta);
      double timeElapsed = currentTime - lastMicros;
      if (!decelerating) {
        desiredVelocity = turnAccelConst[bottleCount] * 1000000.0 / timeElapsed;
        if (brakingDistance(rotationVelocity, 0, turnDecelConst[bottleCount]) >= dTheta) {
          accelerating = false;
          decelerating = true;
          startVelo = rotationVelocity;
          startAng = dTheta;
        } else if (desiredVelocity >= turnVelocityConst[bottleCount]) {
          desiredVelocity = turnVelocityConst[bottleCount];
          accelerating = false;
        }
      }
      if (decelerating){
        desiredVelocity = accelHandle(startVelo, 0, startAng, 0, dTheta);
      }
      pivotPID.Compute();
      motors.setSpeeds(turnCommand, -turnCommand);
      if (fabs(dTheta) < 0.5 && fabs(rotationVelocity) < 1) {
        motors.setSpeeds(0, 0);
        break;
      }
      printStuff();
      lastMicros = currentTime;
    }
  }
}*/

void printStuff() {
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
    Serial.println(turnCommand);
    lastTimer = micros();
  }
  count++;
}

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
  }
  gyroInit = heading.yaw;
  posePrevMicros = turnPrevMicros = fwdPrevMicros = micros();
}
void loop() {
  /*if (rvc.read(&heading)) {
    callLoop();
    doTheThing();
    if (distanceToTarget <= startTurn) {
      moveIndex++;
      gyroInit += .35;
      if (moveIndex > 3) {
        moveIndex = 0;
      }
    }
    }*/
  pivot();
  while (!buttonA.isPressed()) {
    callLoop();
  }
}