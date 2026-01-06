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
double comX;
double comY;
double comTheta;
int moveCount = 0;

//-Movement Vars-
//double endPower[] = { 150, 150, 150, 150 };
int moveIndex = 0;
double maxDelta = 5;
double veloConst = .5;  // HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

//-Pose Vars-
double theta = 0.0;
double xPos = 0.0;
double yPos = 0.0;  //-.35
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
int pauseCount = -1;
double pauseLength = .8824;  //HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEe
double targetTime = 76;

//-Pose Calculation Volatiles-
//Dist
double gyroInit = 0.0;
//Timer
unsigned long currentTime = 0;

//Zero definition...
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

//Parent Functions
__attribute__((noinline)) void pivot(float deg = 180.0, double rotVeloTarg = 90, bool toDefAngle = false) {
  comTheta += deg;
  if (comTheta > 270) {
    comTheta -= 360;
  } else if (comTheta < 0) {
    comTheta += 360;
  }
  // Desired absolute heading (wrapped)
  rotVeloTarg = fabs(rotVeloTarg);
  if (toDefAngle) {
    desiredTheta = wrap180(deg);
    if (wrap180(deg-theta) < 0) {
      rotVeloTarg = -rotVeloTarg;
    }
  } else {
    desiredTheta = wrap180(theta + deg);
    rotVeloTarg = copysign(rotVeloTarg, deg);
  }
  maxDelta = 5;

  // Local PID I/O variables (fresh for each pivot)
  double velocityOutput = 0.0;
  double veloSet;
  bool slowing = false;
  bool accel = true;
  double decelStartSpeed;
  double decelStartPos;

  unsigned long startMicros = micros();

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
      veloSet = accelHandle(decelStartSpeed, 0, decelStartPos, 0, dTheta);
    }
    // compute PID
    veloPID.Compute();

    double out = lastOutput + velocityOutput;
    out = constrain(out, -maxPower, maxPower);
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
      pauseCount++;
      delay(pauseLength * 1000);
      break;
    }
  }
}

__attribute__((noinline)) void seek(double x, double y, double endVelo = 0.02, double endDist = .01, bool varVelo = false, double veloTarg = .5, bool endMove = true, bool reverse = false) {
  xTarget = x;
  yTarget = y;
  if (reverse) {
    endVelo = -endVelo;
    veloTarg = -veloTarg;
  }
  maxDelta = 15;
  while (!rvc.read(&heading)) {
    yield();
  }
  updatePose();
  calculateNextTarget(reverse);
  double velocityOutput = 0.0;
  double rotOutput = 0.0;
  double veloSet = velocity;
  bool slowing = false;
  bool accel = false;
  bool turning = true;
  double decelStartSpeed;
  double decelStartPos;
  int leftTravelTotal = 0;
  int rightTravelTotal = 0;

  //PID Creation
  PID veloPID(&velocity, &velocityOutput, &veloSet, 30, 0, 0.1, DIRECT);
  veloPID.SetOutputLimits(-maxDelta, maxDelta);
  veloPID.SetSampleTime(5);  // ms - reasonable control loop
  veloPID.SetMode(AUTOMATIC);
  PID rotPID(&dTheta, &rotOutput, &zero, 10, 0, 0.3, REVERSE);
  double turnPow = 100;
  if (bottleCount >= 1) {
    turnPow = 150;
  }
  rotPID.SetOutputLimits(-turnPow, turnPow);
  rotPID.SetSampleTime(5);
  rotPID.SetMode(AUTOMATIC);
  unsigned long startMicros;
  double startVelo = velocity;
  while (true) {

    // update odometry/measurements
    updatePose();
    calculateNextTarget(reverse);
    leftTravelTotal += leftTravel;
    rightTravelTotal += rightTravel;
    if (turning){
      veloSet = startVelo;
    }
    if (!accel && turning && fabs(dTheta)<20 ){
      accel = true;
      turning = false;
      startMicros = micros();
    }
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
    double rotAdjustment;
    veloPID.Compute();
    if (distanceToTarget >= 0.1 && fabs(dTheta) > 20) {  //If more than 0.1m off targ, and 20 off target angle, then do fixed arc P
      if ((dTheta > 0 && !reverse) || (dTheta < 0 && reverse)) {
        rotAdjustment = (1.63 * leftTravel - rightTravel) * 3;
      } else {
        rotAdjustment = (leftTravel - 1.63 * rightTravel) * 3;
      }
    } else {  //else regular angular PID
      rotPID.Compute();
      rotAdjustment = rotOutput;
    }
    double out = lastOutput + velocityOutput;
    out = constrain(out, -maxPower, maxPower);
    lastOutput = out;

    motors.setSpeeds(out + rotAdjustment, out - rotAdjustment);

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

/*//Convinience Wrappers
__attribute__((noinline)) void forward(double dist) {
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

__attribute__((noinline)) void forwardT(double dist) {
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

__attribute__((noinline)) void left(double dist) {
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

__attribute__((noinline)) void leftT(double dist) {
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

__attribute__((noinline)) void right(double dist) {
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

__attribute__((noinline)) void rightT(double dist) {
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

__attribute__((noinline)) void back(double dist) {
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
  seek(comX, comY, 0.02, 0.01, false, veloConst, true, true);
  unsigned long end = micros();
  moveTimes[moveCount++] = ((double)(end - start) / 1e6);
  pauseCount++;
  delay(pauseLength * 1000);
}

__attribute__((noinline)) void forwardEnd(double dist) {
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
  pauseCount++;
  delay(pauseLength * 1000);
}

__attribute__((noinline)) void getBottle() {
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

__attribute__((noinline)) void getBottleEnd() {
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
  pauseCount++;
  delay(pauseLength * 1000);
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
}
void loop() {
  unsigned long startRun = micros();
  /*seek(0, -.5, .2, .15, false, .5, false, true);
  seek(-.5, -.5, .02, .01, false, .5, true, true);
  pivot(90);*/
  xPos = 0.9850;
  yPos = -0.3365;
  seek(1.00, 0.50, 0.02, 0.01, false, veloConst, true);
  //forwardEnd
  seek(1.00, 0.00, 0.2, 0.15, false, veloConst, false, true);
  //backwardT
  seek(0.50, -0.00, 0.02, 0.01, false, veloConst, true, true);
  //backwardEnd
  pivot(-90.00);
  pivot(90.00);
  seek(1.00, 0.00, 0.2, 0.15, false, veloConst);
  //forwardT
  seek(1.00, 0.50, 0.02, 0.01, false, veloConst, true);
  //forwardEnd
  //Done with move, errorCount: 0

  unsigned long endRun = micros();
  while (!buttonA.isPressed()) {
    if (buttonB.isPressed()) {
      Serial.print("[");
      for (int i = 0; i < 100; i++) {
        if (moveTimes[i] == 0) break;
        Serial.print(moveTimes[i]);
        Serial.print(", ");
      }
      Serial.println("]");
      Serial.print("Run time: ");
      double runLength = (double)(endRun - startRun) / 1e6;
      Serial.println(runLength);
      Serial.print("Pause count: ");
      Serial.println(pauseCount);
      Serial.print("Recommended pauseLength: ");
      Serial.println(((targetTime - runLength) / pauseCount) + pauseLength);  //75 - 78 = -3 to dec over 10 pauses, -.3 from a 1s pauselength return 0.7s
      delay(1000);
      if (rvc.read(&heading)) {
        updatePose();
        //printStuff();
      }
    }
    //Serial.println("Button Pressed");
  }
}