#include <AxisControl.h>
#include <IkControl.h>
#include <avr/pgmspace.h>
#include <extras/ar4.h>

String inData;
String Alarm;
String function;
String speedViolation = "0";
String flag = "";
String debug = "";

const int debugg = 0;

const int Input32 = 32;
const int Input33 = 33;
const int Input34 = 34;
const int Input35 = 35;
const int Input36 = 36;

const int Output37 = 37;
const int Output38 = 38;
const int Output39 = 39;
const int Output40 = 40;
const int Output41 = 41;

AR4 robot;
IkController controller;
AxisStepper track;
void sendRobotPos()
{
  robot.run();
  // controller.SolveFowardKinematic();
  String sendPos = "A" + String(robot.axis[0].currentPosition(), 3) + "B" + String(robot.axis[1].currentPosition(), 3) + "C" + String(robot.axis[2].currentPosition(), 3) + "D" + String(robot.axis[3].currentPosition(), 3) + "E" + String(robot.axis[4].currentPosition(), 3) + "F" + String(robot.axis[5].currentPosition(), 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + 0;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";
}

void setup()
{
  // run once:
  Serial.begin(9600);

  pinMode(Input32, INPUT_PULLUP);
  pinMode(Input33, INPUT_PULLUP);
  pinMode(Input34, INPUT_PULLUP);
  pinMode(Input35, INPUT_PULLUP);
  pinMode(Input36, INPUT_PULLUP);

  pinMode(Output37, OUTPUT);
  pinMode(Output38, OUTPUT);
  pinMode(Output39, OUTPUT);
  pinMode(Output40, OUTPUT);
  pinMode(Output41, OUTPUT);

  controller.attach(robot, robot.run);
  controller.setDHParams(robot.DHParams);
}

void loop()
{
  robot.run();
  while (Serial.available())
  {
    robot.run(); // Calling incase I'm stuck in while loop
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n')
    {
      inData.trim();
      String function = inData.substring(0, 2);
      inData = inData.substring(2);

      //-----COMMAND TO CLOSE---------------------------------------------------DONE
      if (function == "CL")
      {
        delay(5);
        Serial.end();
      }

      //-----COMMAND TEST LIMIT SWITCHES-----------------------------------------DONE
      if (function == "TL")
      {
        String result;
        for (int i = 0; i < ROBOT_nDOFs; i++)
        {
          result += " J" + String(i + 1) + " = " + (int)digitalRead(robot.calib_pins[i]) + " ";
        }
        delay(5);
        Serial.println(TestLim);
      }

      //-----COMMAND SET TOOL FRAME---------------------------------------------------DONE
      if (function == "TF")
      {
        int TFxStart = inData.indexOf('A');
        int TFyStart = inData.indexOf('B');
        int TFzStart = inData.indexOf('C');
        int TFrzStart = inData.indexOf('D');
        int TFryStart = inData.indexOf('E');
        int TFrxStart = inData.indexOf('F');
        float x = inData.substring(TFxStart + 1, TFyStart).toFloat();
        float y = inData.substring(TFyStart + 1, TFzStart).toFloat();
        float z = inData.substring(TFzStart + 1, TFrzStart).toFloat();
        float rx = inData.substring(TFrzStart + 1, TFryStart).toFloat() * M_PI / 180;
        float ry = inData.substring(TFryStart + 1, TFrxStart).toFloat() * M_PI / 180;
        float rz = inData.substring(TFrxStart + 1).toFloat() * M_PI / 180;

        controller.setToolFrame(x, y, z, rx, ry, rz);
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND SET ENCODERS TO 1000---------------------------------------------DONE
      if (function == "SE")
      {
        for (int i = 0; i < 6; i++)
          robot.encoders[i]->write(1000);
        delay(5);
        Serial.print("Done");
      }

      //-----COMMAND READ ENCODERS---------------------------------------------------DONE
      if (function == "RE")
      {
        String Read = " J1 = " + String(robot.encoders[0]->read()) + "   J2 = " + String(robot.encoders[1]->read()) + "   J3 = " + String(robot.encoders[2]->read()) + "   J4 = " + String(robot.encoders[3]->read()) + "   J5 = " + String(robot.encoders[4]->read()) + "   J6 = " + String(robot.encoders[5]->read());
        delay(5);
        Serial.println(Read);
      }

      //-----COMMAND TO WAIT TIME---------------------------------------------------DONE
      if (function == "WT")
      {
        int WTstart = inData.indexOf('S');
        float WaitTime = inData.substring(WTstart + 1).toFloat();
        int WaitTimeMS = WaitTime * 1000;
        delay(WaitTimeMS);
        Serial.println("Done");
      }

      //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------DONE
      if (function == "JF")
      {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH)
        {
          delay(5);
          Serial.println("T");
        }
        if (digitalRead(IJInputNum) == LOW)
        {
          delay(5);
          Serial.println("F");
        }
      }

      //-----COMMAND SET OUTPUT ON---------------------------------------------------DONE
      if (function == "ON")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND SET OUTPUT OFF---------------------------------------------------DONE
      if (function == "OF")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND TO WAIT INPUT ON---------------------------------------------------DONE
      if (function == "WI")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == LOW)
        {
          delay(100);
        }
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND TO WAIT INPUT OFF---------------------------------------------------DONE
      if (function == "WO")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == HIGH)
        {
          delay(100);
        }
        delay(5);
        Serial.println("Done");
      }









      //-----COMMAND REQUEST POSITION------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "RP")
      {
        sendRobotPos();
      }

      //-----COMMAND ZERO TRACK------------------------------------------------
      //----------------------------------------------------------------------
      if (function == "ZT")
      {
      }

      //-----COMMAND CORRECT POSITION-------------------------------------------
      if (function == "CP")
      {
        correctRobotPos();
      }
      //-----COMMAND SEND POSITION----------------------------------------------
      //------------------------------------------------------------------------
      if (function == "SP")
      {
        int J1angStart = inData.indexOf('A');
        int J2angStart = inData.indexOf('B');
        int J3angStart = inData.indexOf('C');
        int J4angStart = inData.indexOf('D');
        int J5angStart = inData.indexOf('E');
        int J6angStart = inData.indexOf('F');
        int TRstepStart = inData.indexOf('G');

        float J1angValue = ((inData.substring(J1angStart + 1, J2angStart).toFloat());
        float J2angValue = inData.indexOf('B');
        float J3angValue = inData.indexOf('C');
        float J4angValue = inData.indexOf('D');
        float J5angValue = inData.indexOf('E');
        float J6angValue = inData.indexOf('F');
        float TRstepValue = inData.indexOf('G');

        // robot 

        // J1StepM = ((inData.substring(J1angStart + 1, J2angStart).toFloat()) + J1axisLimNeg) * J1StepDeg;
        // J2StepM = ((inData.substring(J2angStart + 1, J3angStart).toFloat()) + J2axisLimNeg) * J2StepDeg;
        // J3StepM = ((inData.substring(J3angStart + 1, J4angStart).toFloat()) + J3axisLimNeg) * J3StepDeg;
        // J4StepM = ((inData.substring(J4angStart + 1, J5angStart).toFloat()) + J4axisLimNeg) * J4StepDeg;
        // J5StepM = ((inData.substring(J5angStart + 1, J6angStart).toFloat()) + J5axisLimNeg) * J5StepDeg;
        // J6StepM = ((inData.substring(J6angStart + 1, TRstepStart).toFloat()) + J6axisLimNeg) * J6StepDeg;
        // TRStepM = inData.substring(TRstepStart + 1).toFloat();
        delay(5);
        Serial.println("Done");
      }

      //----- LIVE CARTESIAN JOG  ----------------------------------------------
      //------------------------------------------------------------------------
      if (function == "LC")
      {
        inData = ""; // Clear recieved buffer
      }

      //----- LIVE JOINT JOG  --------------------------------------------------
      //------------------------------------------------------------------------
      if (function == "LJ")
      {
        inData = ""; // Clear recieved buffer
      }

      //----- LIVE TOOL JOG  --------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LT")
      {
        inData = ""; // Clear recieved buffer
      }

      //----- MOVE J IN JOINTS  -----------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "RJ")
      {
        controller;
        inData = ""; // Clear recieved buffer
      }

      //----- Jog T ----------------------------------------------------------
      //----------------------------------------------------------------------
      if (function == "JT")
      {
        inData = ""; // Clear recieved buffer
      }
      //-----COMMAND HOME POSITION---------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "HM")
      {
        delay(5);
        Serial.print("Done");
        inData = ""; // Clear recieved buffer
      }
      //----- MOVE J ----------------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "MJ")
      {
        controller;
        delay(5);
        Serial.print("Done");
        inData = ""; // Clear recieved buffer
      }

      //----- MOVE A (Arc) ----------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "MA")
      {
        controller;
        delay(5);
        Serial.print("Done");
        inData = ""; // Clear recieved buffer
      }

      //----- MOVE C (Cirlce) ------------------------------------------------
      //----------------------------------------------------------------------
      if (function == "MC")
      {
        controller;
        delay(5);
        Serial.print("Done");
        inData = ""; // Clear recieved buffer
      }

      //----- MOVE L ---------------------------------------------------------
      //----------------------------------------------------------------------
      if (function == "ML")
      {
        controller;
        delay(5);
        Serial.print("Done");
        inData = ""; // Clear recieved buffer
      }

      else
      {
        inData = ""; // Clear recieved buffer
      }
    }
  }
}