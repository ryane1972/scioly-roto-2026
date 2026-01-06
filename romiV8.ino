#include <Romi32U4.h>
#include <math.h>
#include "Adafruit_BNO08x_RVC.h"
#include <PID_v1.h>

//-Robot Info-
const double wheelCircumference = M_PI * 0.07;
const int ticks = 1440;
const double mPerTick = wheelCircumference / ticks;
const int maxPower = 300;
//double trackWidth = 0.141;

//-Configable stuff
double veloConst = .5;  // HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

//-Pose Vars-
double theta = 0.0;
double xPos = 0.0;
double yPos = 0.0;  //-.35
double velocity = 0.0;
double rotationVelocity = 0.0;
double acceleration = 0;
double rotationAcceleration = 0;
int leftTravel;
int rightTravel;
double vMax = 0;
char vMaxSign = 0;
long maxSincePose = 0;

//-Dead Rec Vars-
double xTarget = 0.0;
double yTarget = 0.0;
double desiredTheta = 0;
double dTheta = 0;
double distanceToTarget = 0;

//-Commands-
double lastOutput = 0.0;

//-Pose Calculation Volatiles-
//Dist
double gyroInit = 0.0;
//Timer
unsigned long currentTime = 0;

//Zero definition...
double zero = 0.0;

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

//Accel/Decel Logic
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

//Goofy lil print logic
static inline bool timeToPrint(int counts) {
  static long printCounts = 0;
  printCounts++;
  if (printCounts % counts == 0) {
    return true;
  }
  return false;
}

//Pose & Setpoint handlers
void updatePose() {
  theta = heading.yaw - gyroInit;
  static double thetaLast = 0;
  static long encLeftLast = 0;
  static long encRightLast = 0;
  static long microsLast = 0;
  static double rotationVelocityLast = 0;
  static double velocityLast = 0;
  double midTheta = (theta + thetaLast) / 2;
  long encLeft = encoders.getCountsLeft();
  long encRight = encoders.getCountsRight();
  leftTravel = encLeft - encLeftLast;
  rightTravel = encRight - encRightLast;
  int netTravel = (leftTravel + rightTravel) / 2;
  xPos = xPos + mPerTick * (netTravel)*sin(midTheta * M_PI / 180);
  yPos = yPos + mPerTick * (netTravel)*cos(midTheta * M_PI / 180);
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
  if (currentTime - microsLast > maxSincePose) maxSincePose = currentTime - microsLast;
  thetaLast = theta;
  microsLast = currentTime;
  encLeftLast = encLeft;
  encRightLast = encRight;
  rotationVelocityLast = rotationVelocity;
  //rotationAccelerationLast = rotationAcceleration;
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

//Parent Functions
__attribute__((noinline)) void pivot(float deg = 180.0, double rotVeloTarg = 90, bool toDefAngle = false) {
  // Desired absolute heading (wrapped)
  rotVeloTarg = fabs(rotVeloTarg);
  if (toDefAngle) {
    desiredTheta = wrap180(deg);
    if (wrap180(deg - theta) < 0) {
      rotVeloTarg = -rotVeloTarg;
    }
  } else {
    desiredTheta = wrap180(theta + deg);
    rotVeloTarg = copysign(rotVeloTarg, deg);
  }

  // Local PID I/O variables (fresh for each pivot)
  double velocityOutput = 0.0;
  double veloSet;
  bool slowing = false;
  bool accel = true;
  double decelStartSpeed;
  double decelStartPos;
  double maxDelta = 30;

  unsigned long startMicros = micros();

  //PID Creation
  PID veloPID(&rotationVelocity, &velocityOutput, &veloSet, 0.1, 0, 0.003, DIRECT);
  veloPID.SetOutputLimits(-maxDelta, maxDelta);
  veloPID.SetSampleTime(5);  // ms - reasonable control loop
  veloPID.SetMode(AUTOMATIC);

  // For rate limiting the output change
  lastOutput = 0.0;

  // Small deadband so tiny errors don't jitter motors
  double angleTolerance = 1;           // degrees
  const double velocityTolerance = 5;  // deg/s to ensure nearly stopped
  int leftTravelTotal = 0;
  int rightTravelTotal = 0;
  double endVelo = 3;
  if (fabs(deg) <= 30) {
    endVelo = 1;
    angleTolerance = 2;
  }
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

    //ADDED in V6, timed accel on pivot()?
    if (accel) {
      veloSet = timedAccel(startMicros, micros(), 180, 0, rotVeloTarg);
      if (veloSet == rotVeloTarg) {
        accel = false;
        Serial.print("Accel End");
      }
    }

    //enable slowing phase?
    if (!slowing && brakingDistance(fabs(rotationVelocity), 0, 180.0) >= fabs(dTheta)) {
      slowing = true;
      accel = false;
      decelStartSpeed = rotationVelocity;
      decelStartPos = dTheta;
      /*Serial.println("SLOWING");
      Serial.println(brakingDistance(fabs(rotationVelocity), 0, 180.0));
      Serial.println(dTheta);
      Serial.println(rotationVelocity);*/
    }
    //velo handler for slowing
    if (slowing) {
      veloSet = accelHandle(decelStartSpeed, endVelo, decelStartPos, 0, dTheta);
      veloSet = copysign(veloSet, dTheta);
    }
    // compute PID
    veloPID.Compute();

    double out = lastOutput + velocityOutput;
    out = constrain(out, -maxPower, maxPower);
    lastOutput = out;

    //change for being near targ
    if (fabs(dTheta) < angleTolerance) {
      out = 0.0;
    } else if (fabs(out) < 25) {
      out = copysign(25, out);
    }

    motors.setSpeeds(out - offset, -out - offset);

    //exit condition
    if (fabs(dTheta) < angleTolerance && fabs(rotationVelocity) < velocityTolerance) {
      motors.setSpeeds(0, 0);
      //pauseCount++;
      //delay(pauseLength * 1000);
      break;
    }
  }
}

__attribute__((noinline)) void seek(double x, double y,
                                    double endVelo = 0.02,
                                    double endDist = .01,
                                    bool varVelo = false,
                                    double veloTarg = .5,
                                    bool endMove = false,
                                    bool reverse = false) {
  xTarget = x;
  yTarget = y;

  if (reverse) {
    endVelo = -endVelo;
    veloTarg = -veloTarg;
  }

  double maxDelta = 15;

  while (!rvc.read(&heading)) { yield(); }
  updatePose();
  calculateNextTarget(reverse);

  double velocityOutput = 0.0;
  double rotOutput = 0.0;
  double veloSet = velocity;

  bool slowing = false;
  bool accel = false;
  bool turning = true;
  bool accelDone = false;

  double decelStartSpeed = 0.0;
  double decelStartPos = 0.0;

  //pids
  PID veloPID(&velocity, &velocityOutput, &veloSet, 30, 0, 0.1, DIRECT);
  veloPID.SetOutputLimits(-maxDelta, maxDelta);
  veloPID.SetSampleTime(5);
  veloPID.SetMode(AUTOMATIC);

  PID rotPID(&dTheta, &rotOutput, &zero, 10, 0, 0.1, REVERSE);
  double turnPow = 50;
  rotPID.SetOutputLimits(-turnPow, turnPow);
  rotPID.SetSampleTime(5);
  rotPID.SetMode(AUTOMATIC);

  // Arc constants (15 cm radius to robot center)
  const double TRACK_W = 0.141;                     // meters (use your measured track width)
  const double ARC_R = 0.15;                        // meters
  const double ARC_GAIN = TRACK_W / (2.0 * ARC_R);  // ~0.47 for 0.141/0.15

  // Arc blend thresholds (deg)
  const double ARC_THETA_START = 10.0;  // start blending at 10°
  const double ARC_THETA_FULL = 45.0;   // full arc command by 45° (tune)

  unsigned long startMicros = micros();
  double startVelo = velocity;

  while (true) {
    // --- Turning / accel state machine ---
    if (turning) {
      // Hold a small forward setpoint so it doesn't "pivot" from near-zero start velocity
      double minForward = 0.05;  // tune
      double held = startVelo;
      if (fabs(held) < minForward) held = copysign(minForward, veloTarg);
      veloSet = held;
    }

    // Start accel ONCE when mostly aligned
    if (!accelDone && fabs(dTheta) < 20.0) {
      accel = true;
      turning = false;
      accelDone = true;
      startMicros = micros();
      startVelo = velocity;  // refresh ramp start value at the moment accel begins
    }

    if (accel) {
      veloSet = timedAccel(startMicros, micros(), 0.5, startVelo, veloTarg);
      if (veloSet == veloTarg) {
        accel = false;  // don't restart because accelDone is latched
      }
    }

    // --- Decel logic ---
    if (!slowing && brakingDistance(fabs(velocity), fabs(endVelo), 0.5) >= fabs(distanceToTarget - endDist)) {
      slowing = true;
      accel = false;  // decel overrides accel
      decelStartSpeed = velocity;
      decelStartPos = distanceToTarget;
    }

    if (slowing) {
      veloSet = accelHandle(decelStartSpeed, endVelo, decelStartPos, endDist, distanceToTarget);
    }

    // --- Compute forward command ---
    veloPID.Compute();
    double out = lastOutput + velocityOutput;
    out = constrain(out, -maxPower, maxPower);
    lastOutput = out;

    // --- Compute steering ---
    double rotAdjustment = 0.0;

    // Always compute rotPID so we have a baseline steering term
    rotPID.Compute();
    double rotPidAdj = rotOutput;

    // Enable arc math ONLY when far away AND angular error is >= 10 deg
    if (distanceToTarget >= 0.1 && fabs(dTheta) >= 10.0 && fabs(dTheta <= 60.0)) {
      // Fixed-radius arc steering term (proportional to forward command)
      double arcAdjMag = fabs(out) * ARC_GAIN;

      // With your mixer (L = out + rotAdj, R = out - rotAdj):
      // For dTheta > 0 (need left turn), rotAdj should be negative.
      double turnSign = (dTheta <= 0) ? 1.0 : -1.0;

      // Reverse driving flips steering sign

      double arcAdj = -turnSign * arcAdjMag;

      // Cosine blend weight based on |dTheta| (smooth ramp)
      double adt = fabs(dTheta);
      double t = (adt - ARC_THETA_START) / (ARC_THETA_FULL - ARC_THETA_START);
      t = constrain(t, 0.0, 1.0);
      double w = 0.5 - 0.5 * cos(M_PI * t);  // 0..1

      // Blend: small |dTheta| mostly rotPID, large |dTheta| mostly arc
      rotAdjustment = (1.0 - w) * rotPidAdj + w * arcAdj;
    } else {
      // Near target or small angle: normal heading PID only
      rotAdjustment = rotPidAdj;
      //rotAdjustment = constrain(rotAdjustment, -turnPow, turnPow); // Clamp steering
    }

    motors.setSpeeds(out + rotAdjustment, out - rotAdjustment);

    // Exit
    if (distanceToTarget <= endDist) {
      if (endMove) {
        motors.setSpeeds(0, 0);
        lastOutput = 0.0;
      }
      break;
    }

    // Wait for next IMU sample
    while (!rvc.read(&heading)) { yield(); }
    updatePose();
    calculateNextTarget(reverse);
  }
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
}
void loop() {
  unsigned long startRun = micros();

  xPos = -0.0150;
  yPos = -0.3365;
  //Start Copy 1
  seek(0.0000, 0.5000, 0.2, 0.15, false, veloConst);
  //forwardT
  seek(-0.02, 0.60, 0.2, 0.05, false, .2);
  seek(-0.02, 0.80, 0.2, 0.05, false, .2);
  seek(0.0000, 1.0000, 0.02, 0.01, false, .2, true);
  //getBottleEnd
  seek(0.0000, 0.5000, 0.02, 0.01, false, veloConst, true, true);
  //backwardEnd
  pivot(90.00);
  //Pivot
  seek(0.5000, 0.5000, 0.02, 0.01, false, veloConst, true);
  //forwardEnd
  pivot(105.00);
  //Pivot
  pivot(-15.00);
  //Pivot
  seek(0.5000, 1.0000, 0.02, 0.01, false, veloConst, true, true);
  //backwardEnd
  //ERROR AT MOVE 8 BOTTLE LOSS, REVERSING IN RIGHT POS
  //End copy 1
  //Start copy 2
  pivot(-90.00);
  //Pivot
  seek(0.60, 1.02, 0.2, 0.05, false, .2);
  seek(0.80, 1.02, 0.2, 0.05, false, .2);
  seek(1.0000, 1.0000, 0.2, 0.15, false, .2);
  //getBottleT
  seek(1.0000, 0.0000, 0.02, 0.01, false, veloConst, true);
  //forwardEnd
  pivot(15);
  pivot(-15);
  //releaseBottle
  seek(1.0000, 0.5000, 0.2, 0.15, false, veloConst, false, true);
  //backwardT
  //ERROR AT MOVE 14 REVERSE CONTINUOUS WITH BOTTLE
  //ERROR AT MOVE 14 BOTTLE LOSS, REVERSING IN RIGHT POS
  seek(0.5000, 0.5000, 0.02, 0.01, false, veloConst, true, true);
  //backwardEnd
  //End copy 2
  //Start copy 3
  //Done with move, errorCount: 3 , warningCount: 0

  unsigned long endRun = micros();
  while (!buttonA.isPressed()) {
    if (buttonB.isPressed()) {
      Serial.print("Run time: ");
      double runLength = (double)(endRun - startRun) / 1e6;
      Serial.println(runLength);
      Serial.print("Max time since pose call: ");
      Serial.println(maxSincePose);
      delay(1000);

      /*if (rvc.read(&heading)) {
        updatePose();
      }*/
    }
    //Serial.println("Button Pressed");
  }
}