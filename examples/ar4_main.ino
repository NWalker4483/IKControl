#include <AxisControl.h>
#include <IkControl.h>
#include <avr/pgmspace.h>
#include <extras/ar4.h>

String inData;
String Alarm;
String WristCon;
String function;
String speedViolation = "0";
String flag = "";
String debug = "";

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

void printRobotStatus()
{
  robot.run();
  controller.SolveFowardKinematic();
  String status;
  status += "A" + String(robot.axis[0].currentPosition(), 3);
  status += "B" + String(robot.axis[1].currentPosition(), 3);
  status += "C" + String(robot.axis[2].currentPosition(), 3);
  status += "D" + String(robot.axis[3].currentPosition(), 3);
  status += "E" + String(robot.axis[4].currentPosition(), 3);
  status += "F" + String(robot.axis[5].currentPosition(), 3);
  status += "G" + String(controller.xyzuvw_Out[0], 3);
  status += "H" + String(controller.xyzuvw_Out[1], 3);
  status += "I" + String(controller.xyzuvw_Out[2], 3);
  status += "J" + String(controller.xyzuvw_Out[3], 3);
  status += "K" + String(controller.xyzuvw_Out[4], 3);
  status += "L" + String(controller.xyzuvw_Out[5], 3);
  status += "M" + speedViolation;
  status += "N" + debug;
  status += "O" + flag;
  status += "P" + "0";
  delay(5);
  Serial.println(status);
  speedViolation = "0";
  flag = "";
}

void setup()
{
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
  robot.tool.run();
  while (Serial.available())
  {
    robot.run(); // Calling incase I'm stuck in while loop
    robot.tool.run();

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

      //-----COMMAND TEST LIMIT SWITCHES----------------------------------------DONE
      if (function == "TL")
      {
        String result = " ";
        for (int i = 0; i < ROBOT_nDOFs; i++)
        {
          result += "J" + String(i + 1) + " = " + (int)digitalRead(robot.limit_pins[i]) + "  ";
        }
        delay(5);
        Serial.println(result);
      }

      //-----COMMAND ECHO TEST MESSAGE------------------------------------------DONE
      if (function == "TM")
      {
        delay(5);
        Serial.println(inData);
        inData = ""; // Clear recieved buffer
      }

      //-----COMMAND SET TOOL FRAME---------------------------------------------DONE
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

      //-----COMMAND SET ENCODERS TO 1000---------------------------------------DONE
      if (function == "SE")
      {
        for (int i = 0; i < 6; i++)
          robot.encoders[i]->write(1000);
        delay(5);
        Serial.print("Done");
      }

      //-----COMMAND READ ENCODERS----------------------------------------------DONE
      if (function == "RE")
      {
        String result = " ";
        for (int i = 0; i < ROBOT_nDOFs; i++)
        {
          result += "J" + String(i + 1) + " = " + String(robot.encoders[0]->read()) + "  ";
        }
        delay(5);
        Serial.println(result);
      }

      //-----COMMAND TO WAIT TIME-----------------------------------------------DONE
      if (function == "WT")
      {
        int WTstart = inData.indexOf('S');
        float WaitTime = inData.substring(WTstart + 1).toFloat();
        int WaitTimeMS = WaitTime * 1000;
        delay(WaitTimeMS);
        Serial.println("Done");
      }

      //-----COMMAND IF INPUT THEN JUMP-----------------------------------------DONE
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

      //-----COMMAND SET OUTPUT ON----------------------------------------------DONE
      if (function == "ON")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND SET OUTPUT OFF---------------------------------------------DONE
      if (function == "OF")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND TO WAIT INPUT ON-------------------------------------------DONE
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

      //-----COMMAND TO WAIT INPUT OFF------------------------------------------DONE
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

      //-----COMMAND REQUEST POSITION-------------------------------------------DONE
      if (function == "RP")
      {
        printRobotStatus();
        inData = ""; // Clear recieved buffer
      }

      //-----COMMAND CORRECT POSITION-------------------------------------------DONE
      if (function == "CP")
      {
        for (int i = 0; i < ROBOT_nDOFs; i++)
          robot.zero_offsets[i] = 0;
        printRobotStatus();
      }

      //-----COMMAND SEND POSITION---------------------------------------------DONE
      if (function == "SP")
      {
        int J1angStart = inData.indexOf('A');
        int J2angStart = inData.indexOf('B');
        int J3angStart = inData.indexOf('C');
        int J4angStart = inData.indexOf('D');
        int J5angStart = inData.indexOf('E');
        int J6angStart = inData.indexOf('F');
        int TRstepStart = inData.indexOf('G');
        float JangValues[6];
        JangValues[0] = (inData.substring(J1angStart + 1, J2angStart).toFloat());
        JangValues[1] = (inData.substring(J1angStart + 1, J2angStart).toFloat());
        JangValues[2] = (inData.substring(J1angStart + 1, J2angStart).toFloat());
        JangValues[3] = (inData.substring(J1angStart + 1, J2angStart).toFloat());
        JangValues[4] = (inData.substring(J1angStart + 1, J2angStart).toFloat());
        JangValues[5] = (inData.substring(J1angStart + 1, J2angStart).toFloat());
        // float TRstepValue = inData.substring(TRstepStart + 1).toFloat();
        for (int i = 0; i < ROBOT_nDOFs; i++)
        {
          robot.zero_offsets[i] = robot.axis[i].currentPosition() - JangValues[i];
        }
        robot.run();
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND TO CALIBRATE----------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LL")
      {
        // Find Indexes
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int J1calstart = inData.indexOf('G');
        int J2calstart = inData.indexOf('H');
        int J3calstart = inData.indexOf('I');
        int J4calstart = inData.indexOf('J');
        int J5calstart = inData.indexOf('K');
        int J6calstart = inData.indexOf('L');
        ///
        bool J1req = inData.substring(J1start + 1, J2start).toInt();
        bool J2req = inData.substring(J2start + 1, J3start).toInt();
        bool J3req = inData.substring(J3start + 1, J4start).toInt();
        bool J4req = inData.substring(J4start + 1, J5start).toInt();
        bool J5req = inData.substring(J5start + 1, J6start).toInt();
        bool J6req = inData.substring(J6start + 1, J1calstart).toInt();

        float J1calOff = inData.substring(J1calstart + 1, J2calstart).toFloat();
        float J2calOff = inData.substring(J2calstart + 1, J3calstart).toFloat();
        float J3calOff = inData.substring(J3calstart + 1, J4calstart).toFloat();
        float J4calOff = inData.substring(J4calstart + 1, J5calstart).toFloat();
        float J5calOff = inData.substring(J5calstart + 1, J6calstart).toFloat();
        float J6calOff = inData.substring(J6calstart + 1).toFloat();
        // TODO Set Calib Offsets
        robot.calibrate();
      }
      //-----COMMAND ZERO TRACK------------------------------------------------DONE
      if (function == "ZT")
      {
        track.steps_taken = 0;
      }
      //-----COMMAND CALIBRATE TRACK-------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "CT")
      {
        delay(5);
        Serial.print("Done");
      }
      //----- LIVE CARTESIAN JOG  ---------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LC")
      {
        controller;
        inData = ""; // Clear recieved buffer
      }
      //----- LIVE JOINT JOG  -------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LJ")
      {
        controller;
        inData = ""; // Clear recieved buffer
      }
      //----- LIVE TOOL JOG  --------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LT")
      {
        controller;
        inData = ""; // Clear recieved buffer
      }
      //----- MOVE J IN JOINTS  -----------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "RJ")
      {
        controller;
        controller;
        inData = ""; // Clear recieved buffer
      }
      //----- Jog T ----------------------------------------------------------
      //----------------------------------------------------------------------
      if (function == "JT")
      {
        controller;
        inData = ""; // Clear recieved buffer
      }
      //-----COMMAND HOME POSITION---------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "HM")
      {
        controller;
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
      //----- MOVE L (Line) --------------------------------------------------
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