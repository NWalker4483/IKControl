#include <AxisControl.h>
#include <IkControl.h>
#include <avr/pgmspace.h>
#include <extras/ar4.h>

String inData;
String Alarm;
String function;

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

void sendRobotPos()
{
  updatePos();
  String sendPos = "A" + String(robot.axis[0].currentPosition(), 3) + "B" + String(robot.axis[1].currentPosition(), 3) + "C" + String(robot.axis[2].currentPosition(), 3) + "D" + String(robot.axis[3].currentPosition(), 3) + "E" + String(robot.axis[4].currentPosition(), 3) + "F" + String(robot.axis[5].currentPosition(), 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + TRStepM;
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

  controller.attach(robot);
  controller.setDHParams(robot.DHParams);
}

void loop()
{
  robot.run();
  while (Serial.available())
  {
    robot.run();
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n')
    {
      inData.trim();
      String function = inData.substring(0, 2);
      inData = inData.substring(2);

      //-----COMMAND TO CLOSE---------------------------------------------------
      if (function == "CL")
      {
        delay(5);
        Serial.end();
      }

      //-----COMMAND TEST LIMIT SWITCHES---------------------------------------------------
      if (function == "TL")
      {
        // TODO: Rewrite
        String TestLim = " J1 = " + J1calTest + "   J2 = " + J2calTest + "   J3 = " + J3calTest + "   J4 = " + J4calTest + "   J5 = " + J5calTest + "   J6 = " + J6calTest;
        delay(5);
        Serial.println(TestLim);
      }

      //-----COMMAND SET ENCODERS TO 1000---------------------------------------------------
      if (function == "SE")
      {
        for (int i = 0; i < 6; i++)
          robot.encoders[i]->write(1000);
        delay(5);
        Serial.print("Done");
      }

      //-----COMMAND READ ENCODERS---------------------------------------------------
      if (function == "RE")
      {
        String Read = " J1 = " + String(robot.encoders[0]->read()) + "   J2 = " + String(robot.encoders[1]->read()) + "   J3 = " + String(robot.encoders[2]->read()) + "   J4 = " + String(robot.encoders[3]->read()) + "   J5 = " + String(robot.encoders[4]->read()) + "   J6 = " + String(robot.encoders[5]->read());
        delay(5);
        Serial.println(Read);
      }

       //-----COMMAND TO WAIT TIME---------------------------------------------------
      if (function == "WT")
      {
        int WTstart = inData.indexOf('S');
        float WaitTime = inData.substring(WTstart + 1).toFloat();
        int WaitTimeMS = WaitTime * 1000;
        delay(WaitTimeMS);
        Serial.println("Done");
      }

      //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------
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

      //-----COMMAND SET OUTPUT ON---------------------------------------------------
      if (function == "ON")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND SET OUTPUT OFF---------------------------------------------------
      if (function == "OF")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        delay(5);
        Serial.println("Done");
      }

      //-----COMMAND TO WAIT INPUT ON---------------------------------------------------
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

      //-----COMMAND TO WAIT INPUT OFF---------------------------------------------------
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

      //-----COMMAND SEND POSITION---------------------------------------------------
      //-----------------------------------------------------------------------
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

        J1StepM = ((inData.substring(J1angStart + 1, J2angStart).toFloat()) + J1axisLimNeg) * J1StepDeg;
        J2StepM = ((inData.substring(J2angStart + 1, J3angStart).toFloat()) + J2axisLimNeg) * J2StepDeg;
        J3StepM = ((inData.substring(J3angStart + 1, J4angStart).toFloat()) + J3axisLimNeg) * J3StepDeg;
        J4StepM = ((inData.substring(J4angStart + 1, J5angStart).toFloat()) + J4axisLimNeg) * J4StepDeg;
        J5StepM = ((inData.substring(J5angStart + 1, J6angStart).toFloat()) + J5axisLimNeg) * J5StepDeg;
        J6StepM = ((inData.substring(J6angStart + 1, TRstepStart).toFloat()) + J6axisLimNeg) * J6StepDeg;
        TRStepM = inData.substring(TRstepStart + 1).toFloat();
        delay(5);
        Serial.println("Done");
      }

     

      //----- LIVE CARTESIAN JOG  ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LC")
      {
        delay(5);
        Serial.println();

        updatePos();

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        bool JogInPoc = true;
        Alarm = "0";

        int VStart = inData.indexOf("V");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        float Vector = inData.substring(VStart + 1, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = 100;
        float DCCspd = 100;
        float ACCramp = 100;

        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        inData = ""; // Clear recieved buffer

        while (JogInPoc = true)
        {

          xyzuvw_In[0] = xyzuvw_Out[0];
          xyzuvw_In[1] = xyzuvw_Out[1];
          xyzuvw_In[2] = xyzuvw_Out[2];
          xyzuvw_In[3] = xyzuvw_Out[3];
          xyzuvw_In[4] = xyzuvw_Out[4];
          xyzuvw_In[5] = xyzuvw_Out[5];

          if (Vector == 10)
          {
            xyzuvw_In[0] = xyzuvw_Out[0] - 1;
          }
          if (Vector == 11)
          {
            xyzuvw_In[0] = xyzuvw_Out[0] + 1;
          }

          if (Vector == 20)
          {
            xyzuvw_In[1] = xyzuvw_Out[1] - 1;
          }
          if (Vector == 21)
          {
            xyzuvw_In[1] = xyzuvw_Out[1] + 1;
          }

          if (Vector == 30)
          {
            xyzuvw_In[2] = xyzuvw_Out[2] - 1;
          }
          if (Vector == 31)
          {
            xyzuvw_In[2] = xyzuvw_Out[2] + 1;
          }

          if (Vector == 40)
          {
            xyzuvw_In[3] = xyzuvw_Out[3] - 1;
          }
          if (Vector == 41)
          {
            xyzuvw_In[3] = xyzuvw_Out[3] + 1;
          }

          if (Vector == 50)
          {
            xyzuvw_In[4] = xyzuvw_Out[4] - 1;
          }
          if (Vector == 51)
          {
            xyzuvw_In[4] = xyzuvw_Out[4] + 1;
          }

          if (Vector == 60)
          {
            xyzuvw_In[5] = xyzuvw_Out[5] - 1;
          }
          if (Vector == 61)
          {
            xyzuvw_In[5] = xyzuvw_Out[5] + 1;
          }

          SolveInverseKinematic();

          // calc destination motor steps
          int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

          // calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;
          int TRstepDif = 0;

          // determine if requested position is within axis limits
          if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0)))
          {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0)))
          {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0)))
          {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0)))
          {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0)))
          {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0)))
          {
            J6axisFault = 1;
          }
          if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0)))
          {
            TRaxisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;

          // send move command if no axis limit error
          if (TotalAxisFault == 0 && KinematicError == 0)
          {
            resetEncoders();
            driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
            checkEncoders();
            updatePos();
          }

          // stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

          char recieved = Serial.read();
          inData += recieved;
          if (recieved == '\n')
          {
            break;
          }

          // end loop
        }

        // send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0)
        {
          sendRobotPos();
        }
        else if (KinematicError == 1)
        {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else
        {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }

      //----- LIVE JOINT JOG  ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LJ")
      {
        delay(5);
        Serial.println();
        updatePos();

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        bool JogInPoc = true;
        Alarm = "0";

        int VStart = inData.indexOf("V");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        float Vector = inData.substring(VStart + 1, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = 100;
        float DCCspd = 100;
        float ACCramp = 100;

        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        inData = ""; // Clear recieved buffer

        while (JogInPoc = true)
        {

          float J1Angle = JangleIn[0];
          float J2Angle = JangleIn[1];
          float J3Angle = JangleIn[2];
          float J4Angle = JangleIn[3];
          float J5Angle = JangleIn[4];
          float J6Angle = JangleIn[5];
          float xyzuvw_In[6];

          if (Vector == 10)
          {
            J1Angle = JangleIn[0] - .5;
          }
          if (Vector == 11)
          {
            J1Angle = JangleIn[0] + .5;
          }

          if (Vector == 20)
          {
            J2Angle = JangleIn[1] - .5;
          }
          if (Vector == 21)
          {
            J2Angle = JangleIn[1] + .5;
          }

          if (Vector == 30)
          {
            J3Angle = JangleIn[2] - .5;
          }
          if (Vector == 31)
          {
            J3Angle = JangleIn[2] + .5;
          }

          if (Vector == 40)
          {
            J4Angle = JangleIn[3] - .5;
          }
          if (Vector == 41)
          {
            J4Angle = JangleIn[3] + .5;
          }

          if (Vector == 50)
          {
            J5Angle = JangleIn[4] - .5;
          }
          if (Vector == 51)
          {
            J5Angle = JangleIn[4] + .5;
          }

          if (Vector == 60)
          {
            J6Angle = JangleIn[5] - .5;
          }
          if (Vector == 61)
          {
            J6Angle = JangleIn[5] + .5;
          }

          // calc destination motor steps
          int J1futStepM = (J1Angle + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (J2Angle + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (J3Angle + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (J4Angle + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (J5Angle + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (J6Angle + J6axisLimNeg) * J6StepDeg;

          // calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;
          int TRstepDif = 0;

          // determine motor directions
          if (J1stepDif <= 0)
          {
            J1dir = 1;
          }
          else
          {
            J1dir = 0;
          }

          if (J2stepDif <= 0)
          {
            J2dir = 1;
          }
          else
          {
            J2dir = 0;
          }

          if (J3stepDif <= 0)
          {
            J3dir = 1;
          }
          else
          {
            J3dir = 0;
          }

          if (J4stepDif <= 0)
          {
            J4dir = 1;
          }
          else
          {
            J4dir = 0;
          }

          if (J5stepDif <= 0)
          {
            J5dir = 1;
          }
          else
          {
            J5dir = 0;
          }

          if (J6stepDif <= 0)
          {
            J6dir = 1;
          }
          else
          {
            J6dir = 0;
          }

          if (TRstepDif <= 0)
          {
            TRdir = 1;
          }
          else
          {
            TRdir = 0;
          }

          // determine if requested position is within axis limits
          if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0)))
          {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0)))
          {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0)))
          {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0)))
          {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0)))
          {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0)))
          {
            J6axisFault = 1;
          }
          if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0)))
          {
            TRaxisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;

          // send move command if no axis limit error
          if (TotalAxisFault == 0 && KinematicError == 0)
          {
            resetEncoders();
            driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
            checkEncoders();
            updatePos();
          }

          // stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

          char recieved = Serial.read();
          inData += recieved;
          if (recieved == '\n')
          {
            break;
          }

          // end loop
        }

        // send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0)
        {
          sendRobotPos();
        }
        else if (KinematicError == 1)
        {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else
        {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }

      //----- LIVE TOOL JOG  ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LT")
      {
        delay(5);
        Serial.println();

        updatePos();

        int J1axisFault = 0;
        int J2axisFault = 0;
        int J3axisFault = 0;
        int J4axisFault = 0;
        int J5axisFault = 0;
        int J6axisFault = 0;
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        float Xtool = Robot_Kin_Tool[0];
        float Ytool = Robot_Kin_Tool[1];
        float Ztool = Robot_Kin_Tool[2];
        float RZtool = Robot_Kin_Tool[3];
        float RYtool = Robot_Kin_Tool[4];
        float RXtool = Robot_Kin_Tool[5];

        bool JogInPoc = true;
        Alarm = "0";

        int VStart = inData.indexOf("V");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        float Vector = inData.substring(VStart + 1, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = 100;
        float DCCspd = 100;
        float ACCramp = 100;

        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        inData = ""; // Clear recieved buffer

        while (JogInPoc = true)
        {

          Xtool = Robot_Kin_Tool[0];
          Ytool = Robot_Kin_Tool[1];
          Ztool = Robot_Kin_Tool[2];
          RXtool = Robot_Kin_Tool[3];
          RYtool = Robot_Kin_Tool[4];
          RZtool = Robot_Kin_Tool[5];

          if (Vector == 10)
          {
            Robot_Kin_Tool[0] = Robot_Kin_Tool[0] - 1;
          }
          if (Vector == 11)
          {
            Robot_Kin_Tool[0] = Robot_Kin_Tool[0] + 1;
          }

          if (Vector == 20)
          {
            Robot_Kin_Tool[1] = Robot_Kin_Tool[1] - 1;
          }
          if (Vector == 21)
          {
            Robot_Kin_Tool[1] = Robot_Kin_Tool[1] + 1;
          }

          if (Vector == 30)
          {
            Robot_Kin_Tool[2] = Robot_Kin_Tool[2] - 1;
          }
          if (Vector == 31)
          {
            Robot_Kin_Tool[2] = Robot_Kin_Tool[2] + 1;
          }

          if (Vector == 60)
          {
            Robot_Kin_Tool[3] = Robot_Kin_Tool[3] - 1 * M_PI / 180;
          }
          if (Vector == 61)
          {
            Robot_Kin_Tool[3] = Robot_Kin_Tool[3] + 1 * M_PI / 180;
          }

          if (Vector == 50)
          {
            Robot_Kin_Tool[4] = Robot_Kin_Tool[4] - 1 * M_PI / 180;
          }
          if (Vector == 51)
          {
            Robot_Kin_Tool[4] = Robot_Kin_Tool[4] + 1 * M_PI / 180;
          }

          if (Vector == 40)
          {
            Robot_Kin_Tool[5] = Robot_Kin_Tool[5] - 1 * M_PI / 180;
          }
          if (Vector == 41)
          {
            Robot_Kin_Tool[5] = Robot_Kin_Tool[5] + 1 * M_PI / 180;
          }

          JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
          JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
          JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
          JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
          JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
          JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

          xyzuvw_In[0] = xyzuvw_Out[0];
          xyzuvw_In[1] = xyzuvw_Out[1];
          xyzuvw_In[2] = xyzuvw_Out[2];
          xyzuvw_In[3] = xyzuvw_Out[3];
          xyzuvw_In[4] = xyzuvw_Out[4];
          xyzuvw_In[5] = xyzuvw_Out[5];

          SolveInverseKinematic();

          Robot_Kin_Tool[0] = Xtool;
          Robot_Kin_Tool[1] = Ytool;
          Robot_Kin_Tool[2] = Ztool;
          Robot_Kin_Tool[3] = RXtool;
          Robot_Kin_Tool[4] = RYtool;
          Robot_Kin_Tool[5] = RZtool;

          // calc destination motor steps
          int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

          // calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;
          int TRstepDif = 0;

          // determine motor directions
          if (J1stepDif <= 0)
          {
            J1dir = 1;
          }
          else
          {
            J1dir = 0;
          }

          if (J2stepDif <= 0)
          {
            J2dir = 1;
          }
          else
          {
            J2dir = 0;
          }

          if (J3stepDif <= 0)
          {
            J3dir = 1;
          }
          else
          {
            J3dir = 0;
          }

          if (J4stepDif <= 0)
          {
            J4dir = 1;
          }
          else
          {
            J4dir = 0;
          }

          if (J5stepDif <= 0)
          {
            J5dir = 1;
          }
          else
          {
            J5dir = 0;
          }

          if (J6stepDif <= 0)
          {
            J6dir = 1;
          }
          else
          {
            J6dir = 0;
          }

          if (TRstepDif <= 0)
          {
            TRdir = 1;
          }
          else
          {
            TRdir = 0;
          }

          // determine if requested position is within axis limits
          if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0)))
          {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0)))
          {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0)))
          {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0)))
          {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0)))
          {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0)))
          {
            J6axisFault = 1;
          }
          if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0)))
          {
            TRaxisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;

          // send move command if no axis limit error
          if (TotalAxisFault == 0 && KinematicError == 0)
          {
            resetEncoders();
            driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
            checkEncoders();
            updatePos();
          }

          // stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

          char recieved = Serial.read();
          inData += recieved;
          if (recieved == '\n')
          {
            break;
          }

          // end loop
        }

        // send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0)
        {
          sendRobotPos();
        }
        else if (KinematicError == 1)
        {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else
        {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }

      //----- MOVE J IN JOINTS  ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "RJ")
      {
        int TRaxisFault = 0;
        int TotalAxisFault = 0;

        int J1stepStart = inData.indexOf("A");
        int J2stepStart = inData.indexOf("B");
        int J3stepStart = inData.indexOf("C");
        int J4stepStart = inData.indexOf("D");
        int J5stepStart = inData.indexOf("E");
        int J6stepStart = inData.indexOf("F");
        int tStart = inData.indexOf("G");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        float J1Angle;
        float J2Angle;
        float J3Angle;
        float J4Angle;
        float J5Angle;
        float J6Angle;
        float xyzuvw_In[6];
        float railpos;

        J1Angle = inData.substring(J1stepStart + 1, J2stepStart).toFloat();
        J2Angle = inData.substring(J2stepStart + 1, J3stepStart).toFloat();
        J3Angle = inData.substring(J3stepStart + 1, J4stepStart).toFloat();
        J4Angle = inData.substring(J4stepStart + 1, J5stepStart).toFloat();
        J5Angle = inData.substring(J5stepStart + 1, J6stepStart).toFloat();
        J6Angle = inData.substring(J6stepStart + 1, tStart).toFloat();
        xyzuvw_In[6] = inData.substring(tStart + 1, SPstart).toFloat();
        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
        float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
        float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();

        int J1futStepM = (J1Angle + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (J2Angle + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (J3Angle + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (J4Angle + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (J5Angle + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (J6Angle + J6axisLimNeg) * J6StepDeg;
        int TRfutStepM = (xyzuvw_In[6] + TRaxisLimNeg) * TRStepDeg;

        // calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int TRstepDif = TRStepM - TRfutStepM;

        // determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0)))
        {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0)))
        {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0)))
        {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0)))
        {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0)))
        {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0)))
        {
          J6axisFault = 1;
        }
        if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0)))
        {
          TRaxisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;

        // send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0)
        {
          resetEncoders();
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          checkEncoders();
          sendRobotPos();
        }
        else if (KinematicError == 1)
        {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else
        {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
        ////////MOVE COMPLETE///////////
      }

      //----- Jog T ----------------------------------------------------------
      if (function == "JT")
      {
        inData = ""; // Clear recieved buffer
      }

      //----- MOVE J ---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "MJ")
      {     
        int TotalAxisFault = 0;

        Alarm = "0";

        int xStart = inData.indexOf("X");
        int yStart = inData.indexOf("Y");
        int zStart = inData.indexOf("Z");
        int rzStart = inData.indexOf("Rz");
        int ryStart = inData.indexOf("Ry");
        int rxStart = inData.indexOf("Rx");
        int tStart = inData.indexOf("Tr");
        int SPstart = inData.indexOf("S");
        int AcStart = inData.indexOf("Ac");
        int DcStart = inData.indexOf("Dc");
        int RmStart = inData.indexOf("Rm");
        int WristConStart = inData.indexOf("W");

        xyzuvw_In[0] = inData.substring(xStart + 1, yStart).toFloat();
        xyzuvw_In[1] = inData.substring(yStart + 1, zStart).toFloat();
        xyzuvw_In[2] = inData.substring(zStart + 1, rzStart).toFloat();
        xyzuvw_In[3] = inData.substring(rzStart + 2, ryStart).toFloat();
        xyzuvw_In[4] = inData.substring(ryStart + 2, rxStart).toFloat();
        xyzuvw_In[5] = inData.substring(rxStart + 2, tStart).toFloat();
        xyzuvw_In[6] = inData.substring(tStart + 2, SPstart).toFloat();

        String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
        float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
        float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
        float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
        float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();

        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        SolveInverseKinematic();

        // calc destination motor steps
        int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;
        int TRfutStepM = (xyzuvw_In[6] + TRaxisLimNeg) * TRStepDeg;

        // calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int TRstepDif = TRStepM - TRfutStepM;

        // determine motor directions
     

        // determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0)))
        {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0)))
        {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0)))
        {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0)))
        {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0)))
        {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0)))
        {
          J6axisFault = 1;
        }
        if ((TRdir == 1 and (TRStepM + TRstepDif > TRStepLim)) or (TRdir == 0 and (TRStepM - TRstepDif < 0)))
        {
          TRaxisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + TRaxisFault;

        // send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0)
        {
          resetEncoders();
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(TRstepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          checkEncoders();
          sendRobotPos();
        }
        else if (KinematicError == 1)
        {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else
        {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(TRaxisFault);
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
      }

      //----- MOVE A (Arc) ----------------------------------------------------
      if (function == "MA")
      {
        inData = ""; // Clear recieved buffer
      }

      //----- MOVE C (Cirlce) -------------------------------------------------
      if (function == "MC")
      {
        inData = ""; // Clear recieved buffer
      }

      //----- MOVE L ----------------------------------------------------------
      if (function == "ML")
      {
        inData = ""; // Clear recieved buffer
      }
      
      else
      {
        inData = ""; // Clear recieved buffer
      }
    }
  }
}