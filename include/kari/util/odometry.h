#include "kari/util/misc.h"
#include "api.h"

extern pros::ADIEncoder LEncoder, REncoder, MEncoder;

class Odom {
  public:

    // Getters & Setters
    int * getL();
    int * getR();
    int * getM();
    int * getDL();
    int * getDR();
    int * getDM();

    double * getThetaDeg();
    double * getThetaRad();
    double * getX();
    double * getY();

    Odom& calibrateGyro();
    Odom& zero();
    Odom& reset();

    static void start(void* ignore);
    void run();
    void stop();

  private:
    static bool isRunning;

    static int currentL, currentR, currentM;
    static int deltaL, deltaR, deltaM, lastDeltaL, lastDeltaR, lastDeltaM;

    static double inertL, inertR, inertT;
    static double thetaRad, thetaDeg, offset, posX, posY;

    static double output, DesiredX, DesiredY, Desiredtheta;
};
