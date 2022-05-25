// VERSION 1.2

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
// TODO: Move to axis control extras lib

#include <AxisControl.h>
#include <extras/axis_stepper.h>
#include <Encoder.h>

#define ROBOT_nDOFs 6
#define CALIB_SPEED 3      // deg/s
#define BACKOFF_DIST 3     // degs
#define OVERDRIVE_TIME .25 // seconds
#define WIGGLE_FACTOR 15   // stepss

// Setup Joint Steppers
AxisStepper stepper1(0, 1);
AxisStepper stepper2(2, 3);
AxisStepper stepper3(4, 5);
AxisStepper stepper4(6, 7);
AxisStepper stepper5(8, 9);
AxisStepper stepper6(10, 11);

// Setup encoders
Encoder encoder1(14, 15);
Encoder encoder2(17, 16);
Encoder encoder3(19, 18);
Encoder encoder4(20, 21);
Encoder encoder5(23, 22);
Encoder encoder6(24, 25);

class AR4 : public MultiAxis<ROBOT_nDOFs>
{

public:
    // degrees from limit switch to offset calibration
    const int limit_pins[ROBOT_nDOFs] = {26, 27, 28, 29, 30, 31};
    // 
    const float limit_dir[ROBOT_nDOFs] = {-1, -1, 1, 1, -1, 1};
    // ...offsets for calibration
    float calib_offsets[ROBOT_nDOFs] = {-1, 2, 4.1, -1.5, 3, -7};
    // For modifying the current position without recalibrating
    float zero_offsets[ROBOT_nDOFs] = {0, 0, 0, 0, 0, 0};
    // set to true to prevent the axis's run function from being called
    bool freeze_axis[ROBOT_nDOFs] = {false, false, false, false, false, false};
    // Should the encoder on this axis be used
    bool axis_is_open[ROBOT_nDOFs] = {false, false, false, false, false, false};

    const float DHparams[ROBOT_nDOFs][4] = {
        {0, 0, 169.77, 0},
        {-90, -90, 0, 64.2},
        {0, 0, 0, 305},
        {0, -90, 222.63, 0},
        {0, 90, 0, 0},
        {180, -90, 36.25, 0}};

    AxisStepper *steppers[ROBOT_nDOFs];
    Encoder *encoders[ROBOT_nDOFs];
    // AxisStepper tool(12, 13);
    // AxisStepper track(26, 27);

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

        // Targets are going to be set on the joint level and not the step level
        // steppers[0].disableTargetTracking();
        // steppers[1].disableTargetTracking();
        // steppers[2].disableTargetTracking();
        // steppers[3].disableTargetTracking();
        // steppers[4].disableTargetTracking();
        // steppers[5].disableTargetTracking();

        // Grab Encoder Objects
        encoders[0] = &encoder1;
        encoders[1] = &encoder2;
        encoders[2] = &encoder3;
        encoders[3] = &encoder4;
        encoders[4] = &encoder5;
        encoders[5] = &encoder6;

        // Set Axis Degree Limits
        axis[0].setPoseLimits(-170, 170);
        axis[1].setPoseLimits(-42, 90);
        axis[2].setPoseLimits(-89, 52);
        axis[3].setPoseLimits(-165, 165);
        axis[4].setPoseLimits(-105, 105);
        axis[5].setPoseLimits(-155, 155);

        for (int i = 0; i < ROBOT_nDOFs; i++)
            axis[i].setResolution(WIGGLE_FACTOR * (1L / step_deg[i]));

        // tool.setLimits(0, 3450);
        // tool.setResolution(WIGGLE_FACTOR * (1L / tool_step_deg));

        for (int i = 0; i < ROBOT_nDOFs; i++)
            pinMode(limit_pins[i], INPUT);
    }

    void calibrate()
    {
        // setLimitMode(0); // Run at Constant Speed
        // // TODO:  SET CAL DIRECTION

        // for (int i = 0; i < ROBOT_nDOFs; i++)
        //     axis[i].setTargetSpeed((float)CALIB_SPEED * limit_dir[i]);

        // driveLimits();

        // for (int i = 0; i < ROBOT_nDOFs; i++)
        //     axis[i].move((float)BACKOFF_DIST * limit_dir[i] * -1L);

        // runToPositions();

        // for (int i = 0; i < ROBOT_nDOFs; i++)
        //     axis[i].setTargetSpeed(((float)CALIB_SPEED / 2.) * limit_dir[i]);

        // driveLimits();

        // // Overdrive at half speed
        // disableTargetTracking();
        // unsigned int start_time = millis();
        // unsigned int drive_time = 1000L * OVERDRIVE_TIME;
        // while ((millis() - start_time) < drive_time)
        // {
        //     run()
        // }

        // delay(500);

        // // RESET ENCODERS AND OR STEP COUNTS

        // enableTargetTracking();
        // delay(500);

        resetTracking();
    }

    void driveLimits()
    {
    }

    unsigned int getMillis() { return millis(); };

    void computeAxisPositions(double *axis_positions)
    {
        for (int i = 0; i < 6; i++)
        {
            float steps_taken;
            if (axis_is_open[i])
            {
                steps_taken = steppers[i]->currentPosition();
            }
            else
            {
                steps_taken = (encoders[i]->read() / enc_mult[i]); // Encoder Steps to degress
            }
            *(axis_positions + i) = (steps_taken / step_deg[i]) + zero_offsets[i] + calib_offsets[i];
        }
    };

    void resetTracking(){
        for (int i = 0; i < ROBOT_nDOFs; i++) {
            encoders[i]->write(0);
            steppers[i]->clearSteps();
        } 
    }

    void updateMotorSpeeds(double *axis_speeds)
    {
        for (int i = 0; i < 6; i++)
            steppers[i]->setTargetSpeed(*(axis_speeds + i) * step_deg[i]);
    }

    void pollMotors()
    {
        for (int i = 0; i < 6; i++)
            steppers[i]->pollMotor();
    };
};
