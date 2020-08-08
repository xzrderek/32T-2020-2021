#ifndef LOWLEVEL_H
#define LOWLEVEL_H
#include "api.h"
#include "util.h"
#include <string>
#include <vector>

using namespace pros;

// extern ADIEncoder encoderL, encoderR, encoderM;
extern float encM, encL, encR;

class Position
{
public:
  Position() : X(0), Y(0), heading(0) {}
  Position(float x, float y, float angle) : X(x), Y(y), heading(angle) {}
  float X = 0, Y = 0, heading = 0;
  float distanceToPoint(Position p1)
  {
    return sqrt(sqr(p1.X - X) + sqr(p1.Y - Y));
  }
};

class Odometry
{
public:
  Odometry(Position primary, Position trackers) : pos(primary), t_pos(trackers)
  {
    // pos.X = pos.Y = pos.heading = 0.0;
    // t_pos.X = t_pos.Y = t_pos.heading = 0.0;
    lastR = lastL = lastM = 0;
    resetAngleSentinel = -PI;
    resetEncoders = false;
  }

  Position pos, t_pos;
  volatile bool resetEncoders = false;
  const float wheelWidth = WHEELWIDTH; //(8.75+1.25*2)*1.09;//distance betweenn L&Rtrackers on base (inch)
  float lastL = 0, lastR = 0, lastM = 0, resetAngleSentinel = -PI;
};

class vec3
{
public:
  vec3(float x = 0.0, float y = 0.0, float z = 0.0) : X(x), Y(y), Z(z) {}
  float X, Y, Z;
  vec3 times(float f) { return vec3(X * f, Y * f, Z * f); }
  float distance(vec3 v) { return sqrt(sqr(v.X - X) + sqr(v.Y - Y)); }
  float distanceV3(vec3 v) { return sqrt(sqr(v.X - X) + sqr(v.Y - Y) + sqr(v.Z - Z)); }
  vec3 operator+(vec3 v) { return vec3(X + v.X, Y + v.Y, Z + v.Z); }
};

class PIDcontroller
{
public:
  PIDcontroller(float p, float i, float d, float t, float dT, bool rev, bool run, float r = 1.0) : kP(p), kI(i), kD(d), thresh(t), delayThresh(dT), isReversed(rev), isRunning(run),
                                                                                                   LastError(0), Integral(0), Derivative(0), goal(0), ratio(r)
  {
    ratio = 1.0;
    isRunning = false;
  }
  float kP, kI, kD;
  float error, ratio;
  volatile bool isReversed, isRunning;
  float Integral, Derivative, LastError;
  volatile float thresh, delayThresh, goal;

  void setPID(float p, float i = 0, float d = 0)
  {
    kP = p;
    kI = i;
    kD = d;
  }

  //functions
  void setGoal(float g)
  {
    goal = g;

    //reset
    Integral = 0;
    Derivative = 0;
    LastError = 0;
    isRunning = true;

    // if (abs(g) < 127) {
    //   ratio = 127 / abs(g);
    //   goal = g * ratio;
    // }
  }

  float getGoal()
  {
    return goal;
  }

  float compute(float current, bool isAngle = false)
  {
    if (true)
    {
      current = current * ratio;
      if (!isAngle)
        error = current - goal; //calculate error
      else
        error = normAngle(current - goal); //calculate error

      int dir = 1;
      float power = 0;
      if (isReversed)
        dir = -1;
      const float untilIntegral = thresh * 7; //considered "low threshold"
      // calculate the integral
      if (kI != 0.0)
      { //calculates integral (only at very end)
        if (fabs(error) < untilIntegral)
          Integral += error; //used for averaging the integral amount, later in motor power divided by 25
        else
          Integral = 0.0;
        power += kI * limUpTo(50, Integral);
      }
      else
        Integral = 0.0;
      // calculate the derivative
      if (kD != 0.0)
      {
        Derivative = error - LastError; //change in errors
        LastError = error;
      }
      else
        Derivative = 0.0;
      power += kD * Derivative;
      //final proportional output
      power += kP * error;
      return dir * power;
    }
    return 0;
  }
};
class Mechanism
{
public:
  Mechanism(int typ, std::vector<pros::Motor> m, std::vector<pros::ADIEncoder> e, class PIDcontroller p) : type(typ), mots(m), encs(e), pid(p)
  {
    slow = SLOW_NORMAL;
  };

  //private:
  int type;
  volatile float slow;
  std::vector<pros::Motor> mots;
  std::vector<pros::ADIEncoder> encs;
  float lastVel = 0;
  int upDog = 0;

public: //functions
  class PIDcontroller pid;
  float velocity = 0;

  void reset()
  {
    for (const pros::Motor &m : mots)
    {
      m.tare_position();
    }
  }

  void set_zero_position()
  {
    for (const pros::Motor &m : mots)
    {
      m.set_zero_position(m.get_position());
    }
  }

  float getSensorVal()
  {
    if (!encs.empty())
    {
      float sumEncoders = 0; //average of all encoders in vector list
      for (const pros::ADIEncoder &e : encs)
      {
        sumEncoders += e.get_value();
      }
      return sumEncoders / encs.size(); //returns avg of all encoders in vector list
    }
    else
    {
      float sumMotEncoders = 0; //average of all MOTOR encoders in vector list
      for (const pros::Motor &m : mots)
      {
        sumMotEncoders += m.get_position();
      }
      return sumMotEncoders / mots.size(); //returns avg of all MOTOR encoders in vector list
    }
  }

  void move(float power, float cap = 127)
  {
    // setPIDState(OFF);
    power = clamp(cap, -cap, power);
    for (const pros::Motor &m : mots)
    { //for each motor in mots
      if (type == MEC_TRAY || type == MEC_INTAKE)
        m.move(power / slow);
      else
        m.move(power);
    }
  }

  void toggleSlow()
  {
    if (slow == SLOW_NORMAL)
    {
      slow = SLOW_NORMAL;
      if (type == MEC_INTAKE)
        slow = SLOW_INTAKE;
      else if (type == MEC_TRAY)
        slow = SLOW_TRAY;
    }
    else
    {
      slow = SLOW_NORMAL;
    }
  }

  void setSlow(float ratio)
  {
    slow = ratio;
  }

  float getSlow()
  {
    return slow;
  }

  // void movePID(float power){
  //   for(const pros::Motor& m : mots){//for each motor in mots
  //     m.move(power);
  //   }
  // }

  void moveVel(float vel)
  {
    setPIDState(OFF);
    for (const pros::Motor &m : mots)
    { //for each motor in mots
      m.move_velocity(vel);
    }
  }

  // void moveTo(float goal, float thresh, float power = 127){//simple encoder move
  //   while(abs(getSensorVal() - goal) > thresh){
  //     move(-sign(getSensorVal() - goal) * power);
  //   }
  //   move(0);
  // }

  void moveTo(float goal)
  {
    setPIDGoal(goal);
    setPIDState(ON);
  }

  void moveToUntil(float goal, int wait = 2000, float cap = 127)
  {
    int t = 0;
    setPIDGoal(goal);
    // setPIDState(ON);
    float currentDist = 0;
    while (t < wait)
    {
      currentDist = getSensorVal();
      move(clamp(cap, -cap, pid.compute(currentDist)));
      delay(1);
      t++;
    }
    move(0);
    // setPIDState(OFF);
  }

  void simpleControl(int buttonUp, int buttonDown, int power = 127)
  {
    move(buttonUp * power - buttonDown * power); //simple up down control with 2 buttons (perf for indexer)
  }

  void setPID(float p, float i = 0, float d = 0)
  {
    pid.setPID(p, i, d);
  }

  // void skrrt(int vel){
  //   mots[0].move_velocity(vel);
  //   mots[1].move_velocity(-vel);
  // }

  // void skrrt2(int vel){
  //   mots[0].move_velocity(vel);
  //   mots[1].move(127);
  // }

  // void no(){
  //   move(0);
  // }

  // void no2(){
  //   mots[0].move(0);
  //   mots[1].move(75);
  // }
  //
  // int toggeru(int urMother){ // ask me what upDog is - Uday
  //   if(urMother == 1){
  //
  //     if(upDog == 2){
  //       move(127);
  //       upDog = 0;
  //       delay(300);
  //       return 1;
  //
  //     }
  //     if(upDog == 1){
  //       move(-127);
  //       upDog++;
  //       delay(300);
  //       return 1;
  //     }
  //     if(upDog == 0){
  //       move(0);
  //       upDog++;
  //       delay(300);
  //       return 1;
  //     }
  //
  //   }
  // }

  // void moveAmnt(float amnt, float thresh, float power = 127){//simple encoder move
  //   float starting = getSensorVal();
  //   moveTo(starting + amnt, thresh, power);//moves to the position with AMNT as a constant quantity
  // }

  // void moveTime(float time, float power = 127){//simple time based move
  //   int t = 1;
  //   while (t<time){
  //     move(power);//moves to the position with TIME as a constant quantity
  //     delay(1);
  //     t++;
  //   }
  //   move(-power);
  //   delay(10);
  //   move(0);
  // }

  bool isPIDRunnung()
  {
    return pid.isRunning;
  }

  void PID()
  { //does a PID move HAVE TO SET PID GOAL BEFOREHAND
    if (pid.isRunning)
      move(pid.compute(getSensorVal()));
    return;
  }

  bool setPIDState(bool state)
  { //ON = true, OFF = false
    bool ret = pid.isRunning;
    pid.isRunning = state;
    return ret;
  }

  void setPIDGoal(float amnt)
  {
    pid.setGoal(amnt);
  }

  float getPIDGoal()
  {
    return pid.getGoal();
  }

  // void moveNew(float amnt){
  //   setPIDGoal(amnt);
  //   setPIDState(ON);
  //   int t  = 0;
  //   while(t < 2000){
  //     PID();
  //     delay(1);
  //     t++;
  //   }
  //   setPIDState(OFF);
  // }

  // float computeVel(){//in rots/min
  //   float currentSensor = getSensorVal();
  //   const float delayAmnt = 20;
  //   velocity = ( currentSensor - lastVel) / (delayAmnt / 1000.0);///1000ms in 1 sec
  //   lastVel = currentSensor;
  //   return velocity / 2.0; //(converting ticks/sec to rot/min
  //     //[(ticks/sec) * (60sec/1min) * (1rev/360ticks)] * 3:1 (GR) = (1/6)*3 = 3/6 = 1/2)
  // }

  // float getMotorVel(){
  //   float sumMotVels = 0;//average of all MOTOR encoders in vector list
  //   for(const pros::Motor& m : mots){
  //     sumMotVels += m.get_actual_velocity();
  //   }
  //   return sumMotVels / mots.size();//returns avg of all MOTOR encoders in vector list

  // }
};

// class ChassisNoOdom : public Mechanism {
// public:
//     ChassisNoOdom(std::vector<pros::Motor> m, std::vector<pros::ADIEncoder>e, class PIDcontroller p) : Mechanism(m, e, p) {
//     }
//
//     float yeet(float t){
//       float power = 2;
//       if(sign(t) > 0) return pow(t, power) / pow(127.0, power - 1.0);
//       return -pow(t, power) / pow(127.0, power - 1.0);
//     }
//
//     void driveLR(int powerR, int powerL) {//low level
//       powerL = clamp(127, -127, yeet(powerL));
//       powerR = clamp(127, -127, yeet(powerR));
//       mots[0].move(-powerR); // port 4 right
//       mots[1].move(powerL);//port 5 left
//       mots[2].move(powerL); //port 6 left
//       mots[3].move(-powerR);//port 7  right
//     }
//
//     void driveArcade(int powerFB, int powerLR) {
//       powerLR = clamp(50, -50, yeet(powerLR)) / gSlow;
//       powerFB = clamp(127, -127, yeet(powerFB)) / gSlow;
//       mots[0].move(powerLR - powerFB); //port 4 right
//       mots[1].move(powerLR + powerFB); //port 5 left
//       mots[2].move(powerLR + powerFB); //port 6 left
//       mots[3].move(powerLR - powerFB); //port 7 right
//     }
//
//     void turn(float goal, float power = 100, float cap = 127) {
//       int t = 0;
//       setPIDGoal(goal);
//       float state = setPIDState(OFF);
//       float currentDist = 0;
//       while(t < 2000){
//         currentDist = mots[0].get_position(); //use first motor as representation
//         float speed = clamp(cap, -cap, pid.compute(currentDist));
//         driveArcade(0, speed);
//         delay(1);
//         t++;
//       }
//       driveArcade(0,0);
//       setPIDState(state);
//     }
//
// };
//
class Chassis
{
public:
  Chassis(std::vector<pros::Motor> m, std::vector<PIDcontroller> p, Odometry o) : mots(m), pids(p), odom(o) {}

  //private:
  std::vector<pros::Motor> mots; //first 2 mots are RIGHT, second two are LEFT
  std::vector<PIDcontroller> pids;
  float lastDriveVel = 0, lastRotVel = 0;
  float driveVel = 0, rotVel = 0;
  class Odometry odom;

  void reset()
  {
    for (const pros::Motor &m : mots)
    {
      m.tare_position();
    }

    odom.pos.X = 0;
    odom.pos.Y = 0;
    odom.pos.heading = 90;
  }

  void setPID(int controller, float p, float i = 0, float d = 0)
  {
    pids[controller].setPID(p, i, d);
  }

  float yeet(float t)
  {
    float power = 2;
    if (sign(t) > 0)
      return pow(t, power) / pow(127.0, power - 1.0);
    return -pow(t, power) / pow(127.0, power - 1.0);
  }

  int thing(int potVal, int potReq)
  {
    return ((potVal - potReq));
  }

  //uncomment for TANK

  // void driveLR(int powerR, int powerL){//low level
  //   powerL = clamp(127, -127, yeet(powerL));
  //   powerR = clamp(127, -127, yeet(powerR));
  //   mots[0].move(-powerR); // port 4 right
  //   mots[1].move(powerL);//port 5 left
  //   mots[2].move(powerL); //port 6 left
  //   mots[3].move(-powerR);//port 7  right
  // }

  void driveArcadeTank(int powerFB, int powerLR)
  {
    powerLR = clamp(100, -100, yeet(powerLR));
    powerFB = clamp(127, -127, yeet(powerFB));
    mots[0].move(powerFB - powerLR); //port 4 right front
    mots[1].move(powerFB + powerLR); //port 5 left front
    mots[2].move(powerFB + powerLR); //port 6 left back
    mots[3].move(powerFB - powerLR); //port 7 right back
  }

  void driveArcadeXDrive(int LeftFB, int LeftLR, int RightLR)
  {
    LeftFB = clamp(127, -127, yeet(LeftFB));
    LeftLR = clamp(127, -127, yeet(LeftLR));
    RightLR = clamp(127, -127, yeet(RightLR));
    mots[0].move(-LeftFB + RightLR + LeftFB); //port 4 right front
    mots[1].move(LeftFB + RightLR + LeftFB); //port 5 left front
    mots[2].move(-LeftFB + RightLR - LeftFB); //port 6 left back
    mots[3].move(LeftFB + RightLR - LeftFB); //port 7 right back
  }

  // void brakeHecka(int potVal){//low level
  //   mots[0].move_velocity(0); // port 4 right
  //   mots[1].move_velocity(0);//port 5 left
  //   mots[2].move_velocity(0); //port 6 left
  //   mots[3].move_velocity(0);//port 7  right
  // }

  void fwdsDrive(int power)
  { //BASE
    driveArcade(power, 0);
  }

  void pointTurn(int power)
  { //turn
    driveArcade(0, power);
  }

  // float computeVel(){
  //TODO: need it?
  // float currentSensor = avg(encoderDistInch(encoderL.get_value()), encoderDistInch(encoderR.get_value()));
  // const float delayAmnt = 20;
  // driveVel = ( currentSensor - lastDriveVel) / (delayAmnt / 1000.0);///1000ms in 1 sec
  // lastDriveVel = currentSensor;
  // return driveVel;//converting inch/sec to ???
  // return 0;
  // }

  // float computeRotVel(){
  //   float currentSensor = odom.pos.heading;
  //   const float delayAmnt = 20;
  //   rotVel = ( currentSensor - lastRotVel) / (delayAmnt / 1000.0);///1000ms in 1 sec
  //   lastRotVel = currentSensor;
  //   return rotVel;//converting degrees/sec to ???
  // }

  void smoothDrive(float speed, const float angle, float sharpness = 1)
  {                         //drive base forwards
    const float scalar = 2; //scalar for rotation
    sharpness += 1;         //parameter is from 0-1, do this addition to make sure it ranges from (1-2) [as explained below]
    speed = clamp(127, -127, speed);
    //for sharpness: 2 is direct point turn, 1 is turning off one side...
    //	it basically is just how much the different sides can be reversed to increase tha sharpness of the curve
    float dirSkew = limUpTo(127 * sharpness, scalar * normAngle(odom.pos.heading - angle));
    driveArcade(speed, dirSkew);
  }

  //useless
  // void smoothDriveToPoint(float X, float Y, float sharpness, bool isBackwards = false){
  //   class Position goal(X, Y, 0);
  //   float error = goal.distanceToPoint(odom.pos);
  //   pids[CURVE].goal = 0;//goal is to have no distance between goal and current
  //   //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(error), -0.916209) + 2.05938);//fancy
  //   pids[CURVE].isRunning = false;
  //   while(error > 3){//kinda bad... can be retuned n' stuff
  //     error = goal.distanceToPoint(odom.pos);
  //     //first compute angle to goal
  //     if(!isBackwards) {//normal turn to angle and drive
  //       float phi = normAngle(toDeg(atan2((goal.Y - odom.pos.Y), (goal.X - odom.pos.X))));
  //       //then drive at that angle
  //       smoothDrive(7.5*error, phi, sharpness);
  //     }
  //     else {//drive to point, but backwards, so back reaches the point first.
  //       float phi = normAngle(180 + toDeg(atan2((goal.Y - odom.pos.Y), (goal.X - odom.pos.X))));
  //       //then drive at that angle
  //       smoothDrive(-7.5*error, phi, sharpness);
  //     }
  //   }
  //   fwds(0, 0);
  //   pids[CURVE].isRunning = false;
  //   return;
  // }

  //big pog useful
  void smoothDriveToPointTIME(float X, float Y, float sharpness, float timevar, bool isBackwards = false)
  {
    class Position goal(X, Y, 0);
    float error = goal.distanceToPoint(odom.pos);
    pids[CURVE].goal = 0; //goal is to have no distance between goal and current
    //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(error), -0.916209) + 2.05938);//fancy
    pids[CURVE].isRunning = false;
    int t = 0;
    while (t < timevar)
    { //kinda bad... can be retuned n' stuff
      error = goal.distanceToPoint(odom.pos);
      //first compute angle to goal
      if (!isBackwards)
      { //normal turn to angle and drive
        float phi = normAngle(toDeg(atan2((goal.Y - odom.pos.Y), (goal.X - odom.pos.X))));
        //then drive at that angle
        smoothDrive(7.5 * error, phi, sharpness);
      }
      else
      { //drive to point, but backwards, so back reaches the point first.
        float phi = normAngle(180 + toDeg(atan2((goal.Y - odom.pos.Y), (goal.X - odom.pos.X))));
        //then drive at that angle
        smoothDrive(-7.5 * error, phi, sharpness);
      }
      t++;
      delay(1);
    }
    fwds(0, 0);
    pids[CURVE].isRunning = false;
    return;
  }

private:
  //higher levels
  void fwdsUntil(float amnt, int cap = 127)
  {
    float initX = odom.pos.X;
    float initY = odom.pos.Y;
    int t = 0;
    pids[DRIVE].setGoal(amnt);
    pids[DRIVE].isRunning = true;
    float currentDist = 0;
    while (t < 2000)
    {
      currentDist = sqrt(sqr(odom.pos.X - initX) + sqr(odom.pos.Y - initY));
      fwdsDrive(clamp(cap, -cap, pids[DRIVE].compute(currentDist)));
      delay(1);
      t++;
    }
    fwdsDrive(0);
    pids[DRIVE].isRunning = false;
  }
  void fwdsNEW(float amnt, int cap = 127)
  {
    const float initEncL = encL;
    const float initEncR = encR;
    int t = 0;
    pids[DRIVE].goal = amnt;
    pids[DRIVE].isRunning = true;
    float currentDist = 0;
    while (t < 2000)
    {
      currentDist = avg(encoderDistInch(encL - initEncL), encoderDistInch(encR - initEncR));
      fwdsDrive(clamp(cap, -cap, pids[DRIVE].compute(currentDist)));
      delay(1);
      t++;
    }
    fwdsDrive(0);
    pids[DRIVE].isRunning = false;
  }

  void fwds(const float amnt, const int timeThresh, float cap = 127)
  { //inches...ew //can TOTALLY use the odometry position vectors rather than encoders... smh
    float initX = odom.pos.X;
    float initY = odom.pos.Y;
    pids[DRIVE].goal = amnt;
    pids[DRIVE].isRunning = true; //TURN ON PID
    //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(amnt), -0.916209) + 2.05938);FANCY
    volatile float currentDist = 0.0;
    while (abs(currentDist - pids[DRIVE].goal) > pids[DRIVE].thresh)
    {
      currentDist = sqrt(sqr(odom.pos.X - initX) + sqr(odom.pos.Y - initY));
      fwdsDrive(clamp(cap, -cap, pids[DRIVE].compute(currentDist)));
      delay(1);
    }
    int t = 0;
    while (t < timeThresh)
    {
      currentDist = sqrt(sqr(odom.pos.X - initX) + sqr(odom.pos.Y - initY));
      fwdsDrive(clamp(cap, -cap, pids[DRIVE].compute(currentDist)));
      delay(1);
      t++;
    }
    pids[DRIVE].isRunning = false;
    //final check and correction
    /*const int minSpeed = 30;//slow speed for robot's slight correction
      while(abs(currentDist - pid[DRIVE].goal) > pid[DRIVE].thresh){
      currentDist = avg(encoderDistInch(encoderL.get_value() - initEncLeft), encoderDistInch(encoderR.get_value()  - initEncRight));
      fwdsDrive(clamp(cap, -cap, -sign(currentDist - pid[DRIVE].goal) * minSpeed));
      }*/
    fwdsDrive(0);
    return;
  }

public:
  void moveToUntil(float amnt, int wait = 2000, int cap = 127)
  {
    const float initEncL = encL;
    const float initEncR = encR;
    int t = 0;
    pids[DRIVE].setGoal(amnt);
    pids[DRIVE].isRunning = true;
    float currentDist = 0;
    while (t < wait)
    {
      currentDist = avg(encoderDistInch(encL - initEncL), encoderDistInch(encR - initEncR));
      fwdsDrive(clamp(cap, -cap, pids[DRIVE].compute(currentDist)));
      delay(1);
      t++;
    }
    fwdsDrive(0);
    pids[DRIVE].isRunning = false;
  }

  void turnUntil(float amnt, int wait = 400, int cap = 127)
  {
    int t = 0;
    pids[ANGLE].setGoal(normAngle(odom.pos.heading + amnt));
    pids[ANGLE].isRunning = true;
    while (t < wait)
    {
      pointTurn(clamp(cap, -cap, pids[ANGLE].compute(odom.pos.heading, true)));
      delay(1);
      t++;
    }
    pointTurn(0);
    pids[ANGLE].isRunning = false;
  }

  void driveToPoint(float x, float y, bool isBackwards = false)
  {
    //first compute angle to goal
    //also divide by 0 is fine bc atan2 has error handling
    float phi = normAngle(toDeg(atan2((y - odom.pos.Y), (x - odom.pos.X))));
    //then compute distance to goal
    float dist = sqrt(sqr(y - odom.pos.Y) + sqr(x - odom.pos.X));
    if (!isBackwards)
    {                  //normal turn to angle and drive
      turnUntil(phi);  //simple point turn
      fwdsUntil(dist); //simple drive forwards
    }
    else
    {                                  //drive to point, but backwards, so back reaches the point first.
      turnUntil(normAngle(phi + 180)); //simple point turn (but backwards)
      fwdsUntil(-dist);                //simple drive forwards
    }
    return;
  }

  // use DRIVE2 PID, and can be parallel with other PIDs.
  void moveTo(float amnt)
  {
    setPIDGoal(amnt);
    setPIDState(ON);
  }

  float getSensorVal()
  {
    float sumMotEncoders = 0; //average of all MOTOR encoders in vector list
    for (const pros::Motor &m : mots)
    {
      sumMotEncoders += m.get_position();
    }
    return sumMotEncoders / mots.size(); //returns avg of all MOTOR encoders in vector list
  }

  bool isPIDRunnung()
  {
    return pids[DRIVE2].isRunning;
  }

  void PID()
  { //does a PID move HAVE TO SET PID GOAL BEFOREHAND
    if (pids[DRIVE2].isRunning)
      fwdsDrive(pids[DRIVE2].compute(getSensorVal()));
    return;
  }

  bool setPIDState(bool state)
  { //ON = true, OFF = false
    bool ret = pids[DRIVE2].isRunning;
    pids[DRIVE2].isRunning = state;
    return ret;
  }

  void setPIDGoal(float amnt)
  {
    pids[DRIVE2].setGoal(amnt);
  }

  float getPIDGoal()
  {
    return pids[DRIVE2].getGoal();
  }
  // void fwdsPID(int cap = 127){
  //   float initX = odom.pos.X;
  //   float initY = odom.pos.Y;
  //   int t = 0;
  //   float currentDist = 0;
  //   while(t < 2){
  //     currentDist = sqrt(sqr(odom.pos.X - initX) + sqr(odom.pos.Y - initY));
  //     // currentDist = avg(encoderDistInch(encL - initEncL), encoderDistInch(encR - initEncR));
  //     fwdsDrive(clamp(cap, -cap, pids[DRIVE].compute(currentDist)));
  //     delay(1);
  //     t++;
  //   }
  //   // fwdsDrive(0);
  //   // pids[DRIVE].isRunning = false;
  // }

  // void turnPID(int cap = 127){
  //   int t = 0;
  //   while(t < 2){
  //     pointTurn(pids[ANGLE].compute(odom.pos.heading, true));
  //     delay(1);
  //     t++;
  //   }
  //   // pointTurn(0);
  //   // pids[ANGLE].isRunning = false;
  // }

  // void setPIDGoal(float x, float y, bool isBackwards = false){
  //   //first compute angle to goal
  //   //also divide by 0 is fine bc atan2 has error handling
  //   float phi = normAngle(toDeg(atan2((y - odom.pos.Y), (x - odom.pos.X))));
  //   //then compute distance to goal
  //   float dist = sqrt(sqr(y - odom.pos.Y) + sqr(x - odom.pos.X));
  //   if(isBackwards) {//normal turn to angle and drive
  //     phi = normAngle(phi + 180);//simple point turn (but backwards)
  //     dist = -dist;//simple drive forwards
  //   }
  //   pids[DRIVE].setGoal(dist);
  //   pids[DRIVE].isRunning = true;
  //   pids[ANGLE].setGoal(odom.pos.heading + phi);
  //   pids[ANGLE].isRunning = true;
  //   return;
  // }

  // void PID(){//does a PID move HAVE TO SET PID GOAL BEFOREHAND
  //   if(pids[DRIVE].isRunning) fwdsPID();
  //   if(pids[ANGLE].isRunning) turnPID();
  //   return;
  // }

  // void setPIDState(bool state){//ON = true, OFF = false
  //   pids[DRIVE].isRunning = state;
  //   pids[ANGLE].isRunning = state;
  //   return;
  // }
};

#endif