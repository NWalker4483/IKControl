//VERSION 1.1

/*  AR4 - Stepper motor robot control software
    Copyright (c) 2021, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling Annin Robotics software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

*/


// VERSION LOG
// 1.0 - 2/6/21 - initial release
// 1.1 - 2/20/21 - bug fix, calibration offset on negative axis calibration direction axis 2,4,5
// 1.2 - 5/17/22 - Refactoring 

#include <AxisControl.h>
#include <extras/accel_stepper.h>
#include <Encoder.h>

#define ROBOT_nDOFs 6

// Setup Steppers
AccelStepper stepper1(0,1);
AccelStepper stepper2(2,3);
AccelStepper stepper3(4,5);
AccelStepper stepper4(6,7);
AccelStepper stepper5(8,9);
AccelStepper stepper6(10,11);

// Setup encoders
Encoder J1encPos(14, 15);
Encoder J2encPos(17, 16);
Encoder J3encPos(19, 18);
Encoder J4encPos(20, 21);
Encoder J5encPos(23, 22);
Encoder J6encPos(24, 25);

#define CALIB_SPEED .5

class AR4 : public MultiAxis<ROBOT_nDOFs>{
    AccelStepper *steppers[ROBOT_nDOFs];
    Axis tool;

    AR4()
    {
        // Grab Stepper Objects
        steppers[0] = &stepper1;
        steppers[1] = &stepper2;
        steppers[2] = &stepper3;
        steppers[3] = &stepper4;
        steppers[4] = &stepper5;
        steppers[5] = &stepper6;

        // Set Axis Degree Limits
        axis[0].setLimits(-170, 170);
        axis[1].setLimits(-42, 90);
        axis[2].setLimits(-89, 52);
        axis[3].setLimits(-165, 165);
        axis[4].setLimits(-105, 105);
        axis[5].setLimits(-155, 155);

        tool.setLimits(0,3450);
    }    

//motor steps per degree
float J1StepDeg = 44.44444444;
float J2StepDeg = 55.55555556;
float J3StepDeg = 55.55555556;
float J4StepDeg = 42.72664356;
float J5StepDeg = 21.86024888;
float J6StepDeg = 22.22222222;
float TRStepDeg = 14.28571429;

//set encoder multiplier
const float J1encMult = 10;
const float J2encMult = 10;
const float J3encMult = 10;
const float J4encMult = 10;
const float J5encMult = 5;
const float J6encMult = 10;


    void driveLimit(){

    }

    void checkEncoders() {
  //read encoders
  J1EncSteps = J1encPos.read() / J1encMult;
  J2EncSteps = J2encPos.read() / J2encMult;
  J3EncSteps = J3encPos.read() / J3encMult;
  J4EncSteps = J4encPos.read() / J4encMult;
  J5EncSteps = J5encPos.read() / J5encMult;
  J6EncSteps = J6encPos.read() / J6encMult;

  if (abs((J1EncSteps - J1StepM)) >= 15) {
    J1collisionTrue = 1;
  }
  if (abs((J2EncSteps - J2StepM)) >= 15) {
    J2collisionTrue = 1;
  }
  if (abs((J3EncSteps - J3StepM)) >= 15) {
    J3collisionTrue = 1;
  }
  if (abs((J4EncSteps - J4StepM)) >= 15) {
    J4collisionTrue = 1;
  }
  if (abs((J5EncSteps - J5StepM)) >= 15) {
    J5collisionTrue = 1;
  }
  if (abs((J6EncSteps - J6StepM)) >= 15) {
    J6collisionTrue = 1;
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }
}

void resetEncoders() {

  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;

  //set encoders to current position
  J1encPos.write(J1StepM * J1encMult);
  J2encPos.write(J2StepM * J2encMult);
  J3encPos.write(J3StepM * J3encMult);
  J4encPos.write(J4StepM * J4encMult);
  J5encPos.write(J5StepM * J5encMult);
  J6encPos.write(J6StepM * J6encMult);

}
}
