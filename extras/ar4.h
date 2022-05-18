// VERSION 1.1

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
#define CALIB_SPEED 1    // deg/s
#define WIGGLE_FACTOR 15 // stepss

// Setup Steppers
AccelStepper stepper1(0, 1);
AccelStepper stepper2(2, 3);
AccelStepper stepper3(4, 5);
AccelStepper stepper4(6, 7);
AccelStepper stepper5(8, 9);
AccelStepper stepper6(10, 11);

AccelStepper stepperT(12, 13);

// Setup encoders
Encoder encoder1(14, 15);
Encoder encoder2(17, 16);
Encoder encoder3(19, 18);
Encoder encoder4(20, 21);
Encoder encoder5(23, 22);
Encoder encoder6(24, 25);

const int calib_pins[ROBOT_nDOFs] = {26, 27, 28, 29, 30, 31};
// degrees from limit switch to offset calibration
const float calib_offsets[ROBOT_nDOFs] = {-1, 2, 4.1, -1.5, 3, -7};

class AR4 : public MultiAxis<ROBOT_nDOFs>
{

public:
    Axis tool;
    bool collision_states[ROBOT_nDOFs];

    // Steps per degree on each axis
    const float step_deg[ROBOT_nDOFs] = {44.44444444,
                                         55.55555556,
                                         55.55555556,
                                         42.72664356,
                                         21.86024888,
                                         22.22222222};

    const float enc_mult[ROBOT_nDOFs] = {10, 10, 10, 10, 5, 10};

    const float tool_step_deg = 14.28571429;

    AR4()
    {
        // Grab Stepper Objects
        steppers[0] = &stepper1;
        steppers[1] = &stepper2;
        steppers[2] = &stepper3;
        steppers[3] = &stepper4;
        steppers[4] = &stepper5;
        steppers[5] = &stepper6;

        // Grab Encoder Objects
        encoders[0] = &encoder1;
        encoders[1] = &encoder2;
        encoders[2] = &encoder3;
        encoders[3] = &encoder4;
        encoders[4] = &encoder5;
        encoders[5] = &encoder6;

        // Set Axis Degree Limits
        axis[0].setLimits(-170, 170);
        axis[1].setLimits(-42, 90);
        axis[2].setLimits(-89, 52);
        axis[3].setLimits(-165, 165);
        axis[4].setLimits(-105, 105);
        axis[5].setLimits(-155, 155);

        for (int i = 0; i < ROBOT_nDOFs; i++)
            axis[i].setResolution(WIGGLE_FACTOR * (1L / step_deg[i]));

        tool.setLimits(0, 3450);
        tool.setResolution(WIGGLE_FACTOR * (1L / tool_step_deg));

        for (int i = 0; i < ROBOT_nDOFs; i++)
            pinMode(calib_pins[i], INPUT);
    }

    void calibrate()
    {
        // SET CAL DIRECTION
        digitalWrite(J1dirPin, HIGH);
        digitalWrite(J2dirPin, HIGH);
        digitalWrite(J3dirPin, LOW);
        digitalWrite(J4dirPin, LOW);
        digitalWrite(J5dirPin, HIGH);
        digitalWrite(J6dirPin, LOW);
        // setLimitMode(-1);

        calib_offsets

        // setLimitMode(0);
    }

    unsigned int getMillis() { return millis(); };

    void computeAxisPositions(double *axis_positions)
    {
        J1EncSteps = encoders[i]->read() / enc_mult[i];

        for (int i = 0; i < 6; i++)
            *(axis_positions) = (steppers[i]->currentPosition() / step_deg[i]);

        // Check for collisions
        int TotalCollision = 0;
        for (int i = 0; i < ROBOT_nDOFs; i++)
            TotalCollision += (int)collision_states[i];

        // if (TotalCollision > 0)
        // {
        //     flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
        // }
    };

    void updateMotorSpeeds(double *axis_speeds)
    {
        for (int i = 0; i < 6; i++)
            *(axis_positions) = (steppers[i]->currentPosition() / step_deg[i]);

        // do the math stuff
        for (int i = 0; i < 6; i++)
            steppers[i]->setTargetSpeed(*(axis_speeds + i));
    }

    void pollMotors()
    {
        for (int i = 0; i < 6; i++)
            steppers[i]->pollMotor();
    };

private:
    AccelStepper *steppers[ROBOT_nDOFs];
    Encoder *encoders[ROBOT_nDOFs];

    void checkEncoders()
    {
        // read encoders
        J1EncSteps = J1encPos.read() / J1encMult;
        J2EncSteps = J2encPos.read() / J2encMult;
        J3EncSteps = J3encPos.read() / J3encMult;
        J4EncSteps = J4encPos.read() / J4encMult;
        J5EncSteps = J5encPos.read() / J5encMult;
        J6EncSteps = J6encPos.read() / J6encMult;

        if (abs((J1EncSteps - J1StepM)) >= 15)
        {
            J1collisionTrue = 1;
        }
        .........
    }

    void resetEncoders()
    {
        // set encoders to current position
        for (int i = 0; i < ROBOT_nDOFs; i++)
            collision_states[i] = false;

        //  for (int i = 0; i < ROBOT_nDOFs; i++) encoders[i]->write();
        J1encPos.write(J1StepM * J1encMult);
        J2encPos.write(J2StepM * J2encMult);
        J3encPos.write(J3StepM * J3encMult);
        J4encPos.write(J4StepM * J4encMult);
        J5encPos.write(J5StepM * J5encMult);
        J6encPos.write(J6StepM * J6encMult);
    }
};
