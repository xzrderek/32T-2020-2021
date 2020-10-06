#include "kari/util/misc.h"
#include "api.h"

extern pros::ADIEncoder LEncoder, REncoder, MEncoder;

class Position
{
public:
  Position() : X(0), Y(0), heading(0) {}
  Position(double x, double y, double angle) : X(x), Y(y), heading(angle) {}
  double X = 0, Y = 0, heading = 0;
  double distanceToPoint(Position p1)
  {
    return sqrt(sqr(p1.X - X) + sqr(p1.Y - Y));
  }
};
class Odom {
  public:

    // Getters & Setters
    int * getL();
    int * getR();
    int * getM();
    int * getDL();
    int * getDR();
    int * getDM();

    double * getLB();
    double * getLF();
    double * getRB();
    double * getRF();

    double * getThetaDeg();
    double * getThetaRad();
    double * getX();
    double * getY();

    // tracking wheel version
    double * getPosX();
    double * getPosY();
    double * getPosHeading();

    Odom& calibrateGyro();
    Odom& zero();
    Odom& reset();

    static void start(void* ignore);
    void run();
    void stop();

  private:
    static bool isRunning;

    static int currentL, currentR, currentM;
    static int deltaL, deltaR, deltaM, lastL, lastR, lastM;
    static double currentLF, currentLB, currentRF, currentRB;
    static double inertL, inertR, inertT;
    static double thetaRad, thetaDeg, lastThetaRad, offset, posX, posY;

    static double output, DesiredX, DesiredY, Desiredtheta;

    static double wheelWidth;

    static Position pos, t_pos;
};
