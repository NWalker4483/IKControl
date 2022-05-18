
#include <math.h>

const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;


//define total axis travel
float J1axisLim = J1axisLimPos + J1axisLimNeg;
float J2axisLim = J2axisLimPos + J2axisLimNeg;
float J3axisLim = J3axisLimPos + J3axisLimNeg;
float J4axisLim = J4axisLimPos + J4axisLimNeg;
float J5axisLim = J5axisLimPos + J5axisLimNeg;
float J6axisLim = J6axisLimPos + J6axisLimNeg;
float TRaxisLim = TRaxisLimPos + TRaxisLimNeg;

//motor steps per degree
float J1StepDeg = 44.44444444;
float J2StepDeg = 55.55555556;
float J3StepDeg = 55.55555556;
float J4StepDeg = 42.72664356;
float J5StepDeg = 21.86024888;
float J6StepDeg = 22.22222222;
float TRStepDeg = 14.28571429;

//steps full movement of each axis
int J1StepLim = J1axisLim * J1StepDeg;
int J2StepLim = J2axisLim * J2StepDeg;
int J3StepLim = J3axisLim * J3StepDeg;
int J4StepLim = J4axisLim * J4StepDeg;
int J5StepLim = J5axisLim * J5StepDeg;
int J6StepLim = J6axisLim * J6StepDeg;
int TRStepLim = TRaxisLim * TRStepDeg;

//step and axis zero
int J1zeroStep = J1axisLimNeg * J1StepDeg;
int J2zeroStep = J2axisLimNeg * J2StepDeg;
int J3zeroStep = J3axisLimNeg * J3StepDeg;
int J4zeroStep = J4axisLimNeg * J4StepDeg;
int J5zeroStep = J5axisLimNeg * J5StepDeg;
int J6zeroStep = J6axisLimNeg * J6StepDeg;

//start master step count at Jzerostep
int J1StepM = J1zeroStep;
int J2StepM = J2zeroStep;
int J3StepM = J3zeroStep;
int J4StepM = J4zeroStep;
int J5StepM = J5zeroStep;
int J6StepM = J6zeroStep;
int TRStepM = TRaxisLimNeg * TRStepDeg;


//degrees from limit switch to offset calibration
float J1calBaseOff = -1;
float J2calBaseOff = 2;
float J3calBaseOff = 4.1;
float J4calBaseOff = -1.5;
float J5calBaseOff = 3;
float J6calBaseOff = -7;
float TRcalBaseOff = 0;

//reset collision indicators
int J1collisionTrue = 0;
int J2collisionTrue = 0;
int J3collisionTrue = 0;
int J4collisionTrue = 0;
int J5collisionTrue = 0;
int J6collisionTrue = 0;
int TotalCollision = 0;
int KinematicError = 0;

float lineDist;

String WristCon;

String Alarm = "0";
String speedViolation = "0";
float maxSpeedDelay = 3000;
float minSpeedDelay = 350;
float linWayDistSP = 2;
String debug = "";
String flag = "";
const int TRACKrotdir = 0;

int J1EncSteps;
int J2EncSteps;
int J3EncSteps;
int J4EncSteps;
int J5EncSteps;
int J6EncSteps;

