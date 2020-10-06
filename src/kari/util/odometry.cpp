#include "kari/util/odometry.h"
#include "kari/control/chassis.h"
#include "api.h"

// extern pros::Motor LF, LB, RF, RB;

pros::ADIEncoder LEncoder(1, 2, true),
                 REncoder(7, 8, true),
                 MEncoder(3, 4);

pros::Imu Imu_L(20), Imu_R(3);

bool Odom::isRunning = false;

int Odom::currentL = 0, Odom::currentR = 0, Odom::currentM = 0;
int Odom::deltaL = 0, Odom::deltaR = 0, Odom::deltaM = 0, Odom::lastL = 0, Odom::lastR = 0, Odom::lastM = 0;
double Odom::currentLF = 0, Odom::currentRF = 0, Odom::currentLB = 0, Odom::currentRB = 0;

double Odom::inertL = 0, Odom::inertR = 0, Odom::inertT = 0;
double Odom::thetaRad = 0, Odom::thetaDeg = 0, Odom::lastThetaRad = 0, Odom::offset = 0, Odom::posX = 0, Odom::posY = 0;

double Odom::output = 0, Odom::Desiredtheta = 0, Odom::DesiredX = 0, Odom::DesiredY = 0;

double Odom::wheelWidth = 14.75; //in inch

Position Odom::pos = Position(0,0,0);
Position Odom::t_pos = Position(0,0,0);

double * Odom::getLF() {
  return &currentLF;
}

double * Odom::getRF() {
  return &currentRF;
}

double * Odom::getLB() {
  return &currentLB;
}

double * Odom::getRB() {
  return &currentRB;
}

int * Odom::getL() {
  return &currentL;
}

int * Odom::getR() {
  return &currentR;
}

int * Odom::getM() {
  return &currentM;
}

int * Odom::getDL() {
  return &deltaL;
}

int * Odom::getDR() {
  return &deltaR;
}

int * Odom::getDM() {
  return &deltaM;
}

double * Odom::getThetaRad() {
  return &thetaRad;
}

double * Odom::getThetaDeg() {
  return &thetaDeg;
}

double * Odom::getX() {
  return &posX;
}

double * Odom::getY() {
  return &posY;
}

double * Odom::getPosX() {
  return &pos.X;
}

double * Odom::getPosY() {
  return &pos.Y;
}

double * Odom::getPosHeading() {
  return &pos.heading;
}

Odom& Odom::calibrateGyro() {
  // Imu_T.reset();
  Imu_L.reset();
  Imu_R.reset();

  while( Imu_L.is_calibrating() || Imu_R.is_calibrating() ) { pros::delay(20); }
  io::master.rumble(" . .");
  return *this;
}

Odom& Odom::zero() {
  float left = abs( Imu_L.get_heading() - 360 ) * PI / 180;
  float right = abs( Imu_R.get_heading() - 360 ) * PI / 180;

  float x = ( cos( left + PI ) + cos( right + PI ) ) / 2;
  float y = ( sin( left + PI ) + sin( right + PI ) ) / 2;

  offset = abs( atan2f(y, x) + PI );
  return *this;
}

Odom& Odom::reset() {
  posX = posY = 0;
  pos.X = pos.Y = 0;
  t_pos.X = t_pos.Y = 0;
  return *this;
}

void Odom::start(void *ignore) {
  if(!isRunning) {
    pros::delay(500);
    Odom *that = static_cast<Odom*>(ignore);
    that -> run();
  }
}

extern double LB_get_position();
extern double LF_get_position();
extern double RB_get_position();
extern double RF_get_position();

void Odom::run() {
  isRunning = true;

  while(isRunning) {
    // Get dx=(deltaL+deltaR)/2 and dy=deltaM
    currentL = LEncoder.get_value();
    currentR = REncoder.get_value();
    currentM = MEncoder.get_value();
    deltaL = currentL- lastL;
    deltaR = currentR - lastR;
    deltaM = currentM - lastM;
    lastL = currentL;
    lastR = currentR;
    lastM = currentM;

    // Use IMU to calculate position
    inertL = abs( Imu_L.get_heading() - 360 ) * PI / 180;
    inertR = abs( Imu_R.get_heading() - 360 ) * PI / 180;
    float x = ( cos( inertL - offset + PI ) + cos( inertR - offset + PI ) ) / 2;
    float y = ( sin( inertL - offset + PI ) + sin( inertR - offset + PI ) ) / 2;
    thetaRad = abs( atan2f(y, x) + PI );
    thetaDeg = thetaRad * 180 / PI;
    float dThetaRad = thetaRad - lastThetaRad; 
    lastThetaRad = thetaRad;
    // Calculate absolute position
    posX = posX + (( deltaL + deltaR ) / 2) * cos( thetaRad );
    posY = posY + (( deltaL + deltaR ) / 2) * sin( thetaRad );

    // Use tracking wheels to calculate position
    // http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
    // https://www-users.cs.umn.edu/~stergios/papers/iros99_pioneer.pdf
    float dL = encoderDistInch(deltaL);
    float dR = encoderDistInch(deltaR);
    float dx = encoderDistInch(deltaM);
    float dy = avg(dR, dL);
    #if 0 // Compute angle 
    float dHeading = boundAngle(toRad((dR - dL) / Odom::wheelWidth;
    float avgHeading = normAngle(pos.heading + dHeading / 2.0);
    float radHeading = boundAngle(toRad(avgHeading));
    // Update current r position.
    t_pos.heading = (t_pos.heading + dHeading); 
    t_pos.X += dx * cos(radHeading) - dy * sin(radHeading);
    t_pos.Y += dx * sin(radHeading) + dy * cos(radHeading);

    //add little vector after calculating H mech's position
    const float distToCenter = (wheelWidth / 2); //inches the center of the H mech to the center of r's rotation
    pos.heading = normAngle(t_pos.heading);
    pos.X = t_pos.X + distToCenter * cos(radHeading) - distToCenter;
    pos.Y = t_pos.Y - distToCenter * sin(radHeading);
    #else // Use IMU angle
    float avgHeading = normAngle(thetaRad - dThetaRad / 2.0);
    float radHeading = boundAngle(toRad(avgHeading));
    const float distToCenter = (wheelWidth / 2);
    t_pos.X += dx * cos(radHeading) - dy * sin(radHeading);
    t_pos.Y += dx * sin(radHeading) + dy * cos(radHeading);
    pos.X = t_pos.X; // + distToCenter * cos(radHeading);
    pos.Y = t_pos.Y; // + distToCenter * sin(radHeading) - distToCenter;
    // std::cout << "odom: radHeading: " << radHeading << " t_pos: " << t_pos.X << ", " << t_pos.Y << std::endl;
    #endif

    // read motor poistions
    currentLB = LB_get_position();
    currentLF = LF_get_position();
    currentRB = RB_get_position();
    currentRF = RF_get_position();

    pros::delay(2); //TODO
  }
}

void Odom::stop() {
  isRunning = false;
}
