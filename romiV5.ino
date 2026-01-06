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
int maxTurnPower = 250;
const int minPower[] = { 30, 50, 75 };
//const float accelConst[] = { 2, 1.75, 1.5 };
//const float decelConst[] = { 2, 2, 2 };
//const float turnAccelConst[] = { 2880, 2880, 2880 };
//const float turnDecelConst[] = { 2880, 2880, 2880 };
const float velocityConst[] = { 1, 1, 1 };
const float turnVelocityConst[] = { 720, 720, 720 };
double startTurn = 0.141;
int turnForwardSpeed = 50;
double maxRotationVelocity = 180.0;
double comX;
double comY;
double comTheta;
int moveCount = 0;

//-Movement Vars-
double xTargets[] = { 0, -0.5, -0.5, 0 };
double yTargets[] = { 1, 1, 0, 0 };
double endPower[] = { 150, 150, 150, 150 };
int moveIndex = 0;
double maxDelta = 5;
double veloConst = .5;  // HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

//-Pose Vars-
double theta = 0.0;
double xPos = 0.0;
double yPos = -.35; //-.35
double velocity = 0.0;
double rotationVelocity = 0.0;
double dTheta = 0;
double distanceToTarget = 0;
double desiredTheta = 0;
double acceleration = 0;
double rotationAcceleration = 0;
int leftTravel;
int rightTravel;
double vMax = 0;
char vMaxSign = 0;

//-Commands-
double turnCommand = 0.0;
double forwardCommand = 0.0;
double totalCommandLeft = 0.0;
double totalCommandRight = 0.0;
double lastOutput = 0.0;
double xTarget = 0.0;
double yTarget = 0.0;

//-Pose Calculation Volatiles-
//Dist
double gyroInit = 0.0;
//Timer
unsigned long currentTime = 0;
unsigned long posePrevMicros = 0, turnPrevMicros = 0, fwdPrevMicros = 0;

double zero = 0.0;

//-Misc Logic-
char bottleCount = 0;
int count = 0;
bool justTurned = false;
double moveTimes[100];
//-Object Definitions-
Romi32U4Motors motors;
Romi32U4Encoders encoders;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
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
  if (fabs(endPoint - startPoint) < .1) return endVelo;
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

static inline double clampToAccel(double startVal, double constraint) {
  double output = startVal;
  double delta = startVal - lastOutput;
  if (fabs(delta) > constraint) {
    output = lastOutput + copysign(constraint, delta);
  }
  return output;
}

static inline double timedAccel(unsigned long startMicros, unsigned long currentMicros, double a, double startVelo, double endVelo) {
  if (startVelo > endVelo && a >= 0) {
    a *= -1;
  } else if (startVelo < endVelo && a <= 0) {
    a *= -1;
  }
  double diff = (double)(currentMicros - startMicros) / 1e6;
  double command = diff * a + startVelo;
  if ((command <= endVelo && a <= 0) || (command >= endVelo && a >= 0)) {
    return (endVelo);
  }
  return (diff * a + startVelo);
}

static inline bool timeToPrint(int counts) {
  static long printCounts = 0;
  if (printCounts % counts == 0) {
    return true;
  }
  return false;
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
  leftTravel = encLeft - encLeftLast;
  rightTravel = encRight - encRightLast;
  int netTravel = (leftTravel + rightTravel) / 2;
  xPos = xPos + mPerTick * (netTravel)*sin(theta * M_PI / 180);
  yPos = yPos + mPerTick * (netTravel)*cos(theta * M_PI / 180);
  currentTime = micros();
  unsigned long timeElapsed = currentTime - microsLast;
  velocity = mPerTick * netTravel * 1000000.0 / timeElapsed;
  float thetaChange = theta - thetaLast;
  if (thetaChange > 180) {                                                     //-178 to 178
    thetaChange = -(thetaLast + 180) /*-2 from -178 to 180*/ - (180 - theta);  //-2 from 180 to 178
  } else if (thetaChange < -180) {                                             // 178 to -178
    thetaChange = (180 - thetaLast) /*2 from 178 to 180*/ + (theta + 180);     //2 from -180 to -178
  }
  rotationVelocity = (thetaChange)*1000000.0 / timeElapsed;
  acceleration = (velocity - velocityLast) * 1000000.0 / timeElapsed;
  rotationAcceleration = (rotationVelocity - rotationVelocityLast) * 1000000.0 / timeElapsed;
  if (fabs(velocity) > vMax) {
    vMax = fabs(velocity);
    if (velocity >= 0) {
      vMaxSign = 1;
    } else {
      vMaxSign = 0;
    }
  }
  thetaLast = theta;
  microsLast = currentTime;
  encLeftLast = encLeft;
  encRightLast = encRight;
}

void calculateNextTarget(bool invert = false) {
  desiredTheta = atan2((xTarget - xPos), (yTarget - yPos)) * 180 / M_PI;
  if (!invert) {
    dTheta = wrap180(desiredTheta - theta);
  } else {
    dTheta = wrap180(desiredTheta - theta + 180);
  }
  distanceToTarget = sqrt(pow(xTarget - xPos, 2) + pow(yTarget - yPos, 2));
}

/*void turnPID(double Kp = 7.5, double Ki = 0.0, double Kd = 0.03) {
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
  //if (fabs(rotationVelocity) > maxRotationVelocity) {
  //  turnCommand = turnCommand * maxRotationVelocity / fabs(rotationVelocity);
  //}
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
}*/

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

/*void doTheThing() {
  turnPID();
  forwardPID();
  static int totalCommandLeftLast = 0;
  static int totalCommandRightLast = 0;
  totalCommandLeft = forwardCommand + turnCommand;
  totalCommandRight = forwardCommand - turnCommand;
  if (fabs(totalCommandLeftLast-totalCommandLeft) >= 15){
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
  }
  totalCommandLeftLast = totalCommandLeft;
  totalCommandRightLast = totalCommandRight;
  motors.setSpeeds(totalCommandLeft, totalCommandRight);
}*/

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
  comTheta += deg;
  if (comTheta > 270) {
    comTheta -= 360;
  } else if (comTheta < 0) {
    comTheta += 360;
  }
  // Desired absolute heading (wrapped)
  desiredTheta = wrap180(theta + deg);
  maxDelta = 5;

  // Local PID I/O variables (fresh for each pivot)
  double velocityOutput = 0.0;
  double veloSet;
  if (bottleCount >= 1) {
    veloSet = copysign(90, deg);
  } else {
    veloSet = copysign(180, deg);
  }
  bool slowing = false;
  double decelStartSpeed;
  double decelStartPos;

  //PID Creation
  PID veloPID(&rotationVelocity, &velocityOutput, &veloSet, 0.1, 0, 0.003, DIRECT);
  veloPID.SetOutputLimits(-maxDelta, maxDelta);
  veloPID.SetSampleTime(5);  // ms - reasonable control loop
  veloPID.SetMode(AUTOMATIC);

  // For rate limiting the output change
  lastOutput = 0.0;

  // Small deadband so tiny errors don't jitter motors
  const double ANGLE_DEADZONE = 1;  // degrees
  const double VEL_TOL = 5;         // deg/s to ensure nearly stopped
  int leftTravelTotal = 0;
  int rightTravelTotal = 0;
  // Main pivot loop
  while (true) {
    if (!rvc.read(&heading)) {
      // wait until next BNO sample
      continue;
    }
    // update odometry/measurements
    updatePose();
    //error
    leftTravelTotal += leftTravel;
    rightTravelTotal += rightTravel;
    int error = leftTravelTotal + rightTravelTotal;
    dTheta = wrap180(desiredTheta - theta);
    int offset = error * 3;  //kP of 3

    //enable slowing phase?
    if (!slowing && brakingDistance(fabs(rotationVelocity), 0, 180.0) >= fabs(dTheta)) {
      slowing = true;
      decelStartSpeed = rotationVelocity;
      decelStartPos = dTheta;
      /*Serial.println("SLOWING");
      Serial.println(brakingDistance(fabs(rotationVelocity), 0, 180.0));
      Serial.println(dTheta);
      Serial.println(rotationVelocity);*/
    }
    //velo handler for slowing
    if (slowing) {
      veloSet = accelHandle(decelStartSpeed, 0, decelStartPos, 0, dTheta);
    }
    // compute PID
    veloPID.Compute();

    double out = lastOutput + velocityOutput;
    out = constrain(out, -300, 300);
    lastOutput = out;

    //change for being near targ
    if (fabs(dTheta) < ANGLE_DEADZONE) {
      out = 0.0;
    } else if (fabs(out) < 25) {
      out = copysign(25, out);
    }

    motors.setSpeeds(out - error, -out - error);

    //exit condition
    if (fabs(dTheta) < ANGLE_DEADZONE && fabs(rotationVelocity) < VEL_TOL) {
      motors.setSpeeds(0, 0);
      break;
    }
  }
}

void seek(double x, double y, double endVelo = 0.1, double endDist = .14, bool varVelo = true, double veloTarg = .5, bool endMove = false) {
  xTarget = x;
  yTarget = y;
  maxDelta = 15;
  while (!rvc.read(&heading)) {
    yield();
  }
  updatePose();
  calculateNextTarget();
  double velocityOutput = 0.0;
  double rotOutput = 0.0;
  double veloSet = velocity;
  bool slowing = false;
  bool accel = true;
  double decelStartSpeed;
  double decelStartPos;

  //PID Creation
  PID veloPID(&velocity, &velocityOutput, &veloSet, 30, 0, 0.1, DIRECT);
  veloPID.SetOutputLimits(-maxDelta, maxDelta);
  veloPID.SetSampleTime(5);  // ms - reasonable control loop
  veloPID.SetMode(AUTOMATIC);
  PID rotPID(&dTheta, &rotOutput, &zero, 10, 0, 0.3, REVERSE);
  double turnPow = 100;
  if (bottleCount>=1){
    turnPow = 150;
  }
  rotPID.SetOutputLimits(-turnPow, turnPow);
  rotPID.SetSampleTime(5);
  rotPID.SetMode(AUTOMATIC);
  unsigned long startMicros = micros();
  double startVelo = velocity;
  while (true) {

    // update odometry/measurements
    updatePose();
    calculateNextTarget();
    if (accel) {
      veloSet = timedAccel(startMicros, micros(), .5, startVelo, veloTarg);
      if (veloSet == veloTarg) {
        accel = false;
        Serial.print("Accel End");
      }
    }
    //enable slowing phase?
    if (!slowing && brakingDistance(fabs(velocity), endVelo, .5) >= fabs(distanceToTarget - endDist)) {
      slowing = true;
      if (accel) {
        Serial.print("Early Accel End");
      }
      accel = false;
      decelStartSpeed = velocity;
      decelStartPos = distanceToTarget;
      Serial.print("Decel Start");
    }
    //velo handler for slowing
    if (slowing) {
      veloSet = accelHandle(decelStartSpeed, endVelo, decelStartPos, endDist, distanceToTarget);
    }
    // compute PID
    veloPID.Compute();
    rotPID.Compute();

    double out = lastOutput + velocityOutput;
    out = constrain(out, -300, 300);
    lastOutput = out;

    motors.setSpeeds(out + rotOutput, out - rotOutput);

    if (distanceToTarget <= endDist) {
      if (endMove) {
        motors.setSpeeds(0, 0);
      }
      break;
    }
    /*if (timeToPrint(10)) {
      Serial.print("VeloSet: ");
      Serial.println(veloSet);
      Serial.print("Velo: ");
      Serial.println(velocity);
      Serial.print("Dist: ");
      Serial.println(distanceToTarget);
      Serial.print("X: ");
      Serial.println(xPos);
      Serial.print("dTheta:");
      Serial.println(dTheta);
      Serial.print("out:");
      Serial.println(out);
      Serial.print("rotOut:");
      Serial.println(rotOutput);
    }*/
    while (!rvc.read(&heading)) {
      yield();
    }
  }
}

void seekBack(double x, double y, double endVelo = -0.1, double endDist = .14, bool varVelo = true, double veloTarg = -.5, bool endMove = false) {
  xTarget = x;
  yTarget = y;
  maxDelta = 15;
  while (!rvc.read(&heading)) {
    yield();
  }
  updatePose();
  calculateNextTarget();
  double velocityOutput = 0.0;
  double rotOutput = 0.0;
  double veloSet = velocity;
  bool slowing = false;
  bool accel = true;
  double decelStartSpeed;
  double decelStartPos;
  unsigned long start = micros();
  //PID Creation
  PID veloPID(&velocity, &velocityOutput, &veloSet, 30, 0, 0.1, DIRECT);
  veloPID.SetOutputLimits(-maxDelta, maxDelta);
  veloPID.SetSampleTime(5);  // ms - reasonable control loop
  veloPID.SetMode(AUTOMATIC);
  PID rotPID(&dTheta, &rotOutput, &zero, 10, 0, 0.3, REVERSE);
  rotPID.SetOutputLimits(-50, 50);
  rotPID.SetSampleTime(5);
  rotPID.SetMode(AUTOMATIC);
  unsigned long startMicros = micros();
  double startVelo = velocity;
  while (true) {

    // update odometry/measurements
    updatePose();
    calculateNextTarget(true);
    if (accel) {
      veloSet = timedAccel(startMicros, micros(), .5, startVelo, veloTarg);
      if (veloSet == veloTarg) {
        accel = false;
        Serial.print("Accel End");
      }
    }
    //enable slowing phase?
    if (!slowing && brakingDistance(fabs(velocity), fabs(endVelo), .5) >= fabs(distanceToTarget - endDist)) {
      slowing = true;
      if (accel) {
        Serial.print("Early Accel End");
      }
      accel = false;
      decelStartSpeed = velocity;
      decelStartPos = distanceToTarget;
      Serial.print("Decel Start");
    }
    //velo handler for slowing
    if (slowing) {
      veloSet = accelHandle(decelStartSpeed, endVelo, decelStartPos, endDist, distanceToTarget);
    }
    // compute PID
    veloPID.Compute();
    rotPID.Compute();

    double out = lastOutput + velocityOutput;
    out = constrain(out, -300, 300);
    lastOutput = out;

    motors.setSpeeds(out + rotOutput, out - rotOutput);

    if (distanceToTarget <= endDist) {
      if (endMove) {
        motors.setSpeeds(0, 0);
      }
      break;
    }
    /*if (timeToPrint(10)) {
      Serial.print("VeloSet: ");
      Serial.println(veloSet);
      Serial.print("Velo: ");
      Serial.println(velocity);
      Serial.print("Dist: ");
      Serial.println(distanceToTarget);
      Serial.print("X: ");
      Serial.println(xPos);
      Serial.print("dTheta:");
      Serial.println(dTheta);
      Serial.print("out:");
      Serial.println(out);
      Serial.print("rotOut:");
      Serial.println(rotOutput);
    }*/
    while (!rvc.read(&heading)) {
      yield();
    }
  }
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void forward(double dist) {
  unsigned long start = micros();
  if (comTheta == 0) {
    comY += dist;
  } else if (comTheta == 90) {
    comX += dist;
  } else if (comTheta == 180) {
    comY -= dist;
  } else if (comTheta == 270) {
    comX -= dist;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seek(comX, comY, veloConst, 0.01, false, veloConst);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void forwardT(double dist) {
  unsigned long start = micros();
  if (comTheta == 0) {
    comY += dist;
  } else if (comTheta == 90) {
    comX += dist;
  } else if (comTheta == 180) {
    comY -= dist;
  } else if (comTheta == 270) {
    comX -= dist;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seek(comX, comY, 0.2, 0.15, false, veloConst);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void left(double dist) {
  comTheta -= 90;
  gyroInit += 0.3;
  if (comTheta == -90) comTheta = 270;
  unsigned long start = micros();
  if (comTheta == 0) {
    comY += dist;
  } else if (comTheta == 90) {
    comX += dist;
  } else if (comTheta == 180) {
    comY -= dist;
  } else if (comTheta == 270) {
    comX -= dist;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seek(comX, comY, veloConst, 0.01, false, veloConst);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void leftT(double dist) {
  comTheta -= 90;
  gyroInit += 0.3;

  if (comTheta == -90) comTheta = 270;
  unsigned long start = micros();
  if (comTheta == 0) {
    comY += dist;
  } else if (comTheta == 90) {
    comX += dist;
  } else if (comTheta == 180) {
    comY -= dist;
  } else if (comTheta == 270) {
    comX -= dist;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seek(comX, comY, 0.2, 0.15, false, veloConst);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void right(double dist) {
  comTheta += 90;
  gyroInit -= 0.3;

  if (comTheta == 360) comTheta = 0;
  unsigned long start = micros();
  if (comTheta == 0) {
    comY += dist;
  } else if (comTheta == 90) {
    comX += dist;
  } else if (comTheta == 180) {
    comY -= dist;
  } else if (comTheta == 270) {
    comX -= dist;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seek(comX, comY, veloConst, 0.01, false, veloConst);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void rightT(double dist) {
  comTheta += 90;
  gyroInit -= 0.3;

  if (comTheta == 360) comTheta = 0;
  unsigned long start = micros();
  if (comTheta == 0) {
    comY += dist;
  } else if (comTheta == 90) {
    comX += dist;
  } else if (comTheta == 180) {
    comY -= dist;
  } else if (comTheta == 270) {
    comX -= dist;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seek(comX, comY, 0.2, 0.15, false, veloConst);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void back(double dist) {
  bottleCount = 0;
  unsigned long start = micros();
  if (comTheta == 0) {
    comY -= dist;
  } else if (comTheta == 90) {
    comX -= dist;
  } else if (comTheta == 180) {
    comY += dist;
  } else if (comTheta == 270) {
    comX += dist;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seekBack(comX, comY, -0.02, 0.01, false, -veloConst, true);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void forwardEnd(double dist) {
  unsigned long start = micros();
  if (comTheta == 0) {
    comY += dist;
  } else if (comTheta == 90) {
    comX += dist;
  } else if (comTheta == 180) {
    comY -= dist;
  } else if (comTheta == 270) {
    comX -= dist;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seek(comX, comY, 0.02, 0.01, false, veloConst, true);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void getBottle() {
  bottleCount = 1;
  unsigned long start = micros();
  if (comTheta == 0) {
    comY += .5;
  } else if (comTheta == 90) {
    comX += .5;
  } else if (comTheta == 180) {
    comY -= .5;
  } else if (comTheta == 270) {
    comX -= .5;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seek(comX, comY, 0.2, 0.01, false, .2);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

void getBottleEnd() {
  bottleCount = 1;
  unsigned long start = micros();
  if (comTheta == 0) {
    comY += .5;
  } else if (comTheta == 90) {
    comX += .5;
  } else if (comTheta == 180) {
    comY -= .5;
  } else if (comTheta == 270) {
    comX -= .5;
  }
  Serial.print("X: ");
  Serial.print(comX);
  Serial.print(" Y: ");
  Serial.print(comY);
  Serial.print(" comTheta: ");
  Serial.println(comTheta);
  seek(comX, comY, 0.02, 0.01, false, .2, true);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
}

/*void contTurn(float deg = 90.0) {

  // Desired absolute heading (wrapped)
  desiredTheta = wrap180(theta + deg);
  maxDelta = 5;

  // Local PID I/O variables (fresh for each pivot)
  double velocityOutput = 0.0;
  double veloSet = copysign(180, deg);

  bool slowing = false;
  double decelStartSpeed;
  double decelStartPos;

  //PID Creation
  PID veloPID(&rotationVelocity, &velocityOutput, &veloSet, 0.1, 0, 0.003, DIRECT);
  veloPID.SetOutputLimits(-maxDelta, maxDelta);
  veloPID.SetSampleTime(5);  // ms - reasonable control loop
  veloPID.SetMode(AUTOMATIC);

  // For rate limiting the output change
  lastOutput = 0.0;

  // Small deadband so tiny errors don't jitter motors
  const double ANGLE_DEADZONE = 0.5;  // degrees
  int leftTravelTotal = 0;
  int rightTravelTotal = 0;
  double higherWheelMulti = 1.6;
  // Main pivot loop
  while (true) {
    if (!rvc.read(&heading)) {
      // wait until next BNO sample
      continue;
    }
    // update odometry/measurements
    updatePose();
    //error
    leftTravelTotal += leftTravel;
    rightTravelTotal += rightTravel;
    if (deg > 0){
      double error = leftTravelTotal - rightTravelTotal*higherWheelMulti;
    }
    dTheta = wrap180(desiredTheta - theta);
    int offset = error * 3;  //kP of 3

    //enable slowing phase?
    if (!slowing && brakingDistance(fabs(rotationVelocity), 0, 180.0) >= fabs(dTheta)) {
      slowing = true;
      decelStartSpeed = rotationVelocity;
      decelStartPos = dTheta;
    }
    //velo handler for slowing
    if (slowing) {
      veloSet = accelHandle(decelStartSpeed, 0, decelStartPos, 0, dTheta);
    }
    // compute PID
    veloPID.Compute();

    double out = lastOutput + velocityOutput;
    out = constrain(out, -300, 300);
    lastOutput = out;

    //change for being near targ
    if (fabs(dTheta) < ANGLE_DEADZONE) {
      out = 0.0;
    } else if (fabs(out) < 25) {
      out = copysign(25, out);
    }

    motors.setSpeeds(out - error, -out - error);

    //exit condition
    if (fabs(dTheta) < ANGLE_DEADZONE) {
      break;
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
  Serial.print("Button Pressed");
  gyroInit = heading.yaw;
  posePrevMicros = turnPrevMicros = fwdPrevMicros = micros();
}
void loop() {
  /*seek(0, 1, 1, 0.1, false, 1);
  seek(-.875, 3, 1, 0.1, false, 1);
  seek(-.875, 5.5 , 1, 0.1, false, 2);
  seek(0,8.63, .01, 0.095, false, 1, true);*/
  //EV
  unsigned long startRun = micros();
  seek(0, 0, 0.5, 0.01, false, .5);  //INIT
  
  forward(.5);
  getBottle();
  rightT(1);
  left(.25);
  forwardEnd(.25);
  back(.5);
  pivot(-90);
  forwardT(1);
  right(.25);
  forwardEnd(.25);
  pivot(90);
  getBottle();
  pivot(-180);
  forwardT(.5);
  left(.25);
  forwardEnd(.25);
  pivot(180);
  back(1);
  pivot(90);
  forwardT(2);
  leftT(.5);
  //getBottle();
  left(.25);
  forwardEnd(.25);
  pivot(-90);
  back(.5);
  pivot(-90);
  forwardT(.5);
  leftT(.5);
  left(.25);
  forwardEnd(.25);
  pivot(180);
  forwardEnd(.5);
  pivot(90);
  forwardEnd(.3);
  unsigned long endRun = micros();
  while (!buttonA.isPressed()) {
    if (buttonB.isPressed()) {
      /*Serial.print(xPos);
      Serial.print(" / ");
      Serial.println(yPos);
      Serial.print("vMax: ");
      if (vMaxSign = 1) {
        Serial.print("+");
      } else {
        Serial.print("-");
      }
      Serial.println(vMax);
    }*/
      Serial.print("[");
      for (int i = 0; i < 100; i++) {
        if (moveTimes[i]==0) break;
        Serial.print(moveTimes[i]);
        Serial.print(", ");
      }
      Serial.println("]");
      Serial.print("Run time: ");
      Serial.println((double)(endRun-startRun)/1e6);
      delay(1000);
      if (rvc.read(&heading)) {
        updatePose();
        //printStuff();
      }
    }
    //Serial.println("Button Pressed");
  }
}