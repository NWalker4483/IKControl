//-----COMMAND CALIBRATE TRACK---------------------------------------------------
if (function == "CT")
{
    int axis7lengthStart = inData.indexOf('A');
    int axis7rotStart = inData.indexOf('B');
    int axis7stepsStart = inData.indexOf('C');
    axis7length = inData.substring(axis7lengthStart + 1, axis7rotStart).toFloat();
    axis7rot = inData.substring(axis7rotStart + 1, axis7stepsStart).toFloat();
    axis7steps = inData.substring(axis7stepsStart + 1).toFloat();
    TRaxisLimNeg = 0;
    TRaxisLimPos = axis7length;
    TRaxisLim = TRaxisLimPos + TRaxisLimNeg;
    TRStepDeg = axis7steps / axis7rot;
    TRStepLim = TRaxisLim * TRStepDeg;
    delay(5);
    Serial.print("Done");
}

//-----COMMAND ZERO TRACK---------------------------------------------------
if (function == "ZT")
{
    TRStepM = 0;
    sendRobotPos();
}

//-----COMMAND SET TOOL FRAME---------------------------------------------------
if (function == "TF")
{
    int TFxStart = inData.indexOf('A');
    int TFyStart = inData.indexOf('B');
    int TFzStart = inData.indexOf('C');
    int TFrzStart = inData.indexOf('D');
    int TFryStart = inData.indexOf('E');
    int TFrxStart = inData.indexOf('F');
    Robot_Kin_Tool[0] = inData.substring(TFxStart + 1, TFyStart).toFloat();
    Robot_Kin_Tool[1] = inData.substring(TFyStart + 1, TFzStart).toFloat();
    Robot_Kin_Tool[2] = inData.substring(TFzStart + 1, TFrzStart).toFloat();
    Robot_Kin_Tool[3] = inData.substring(TFrzStart + 1, TFryStart).toFloat() * M_PI / 180;
    Robot_Kin_Tool[4] = inData.substring(TFryStart + 1, TFrxStart).toFloat() * M_PI / 180;
    Robot_Kin_Tool[5] = inData.substring(TFrxStart + 1).toFloat() * M_PI / 180;
}


     //-----COMMAND HOME POSITION---------------------------------------------------
      if (function == "HM")
      {
      }

     //-----COMMAND REQUEST POSITION---------------------------------------------------
      if (function == "RP")
      {
        sendRobotPos();
      }

 
      //-----COMMAND CORRECT POSITION---------------------------------------------------
      if (function == "CP")
      {
        correctRobotPos();
      }

 //-----COMMAND ECHO TEST MESSAGE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "TM")
      {
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int WristConStart = inData.indexOf('W');
        JangleIn[0] = inData.substring(J1start + 1, J2start).toFloat();
        JangleIn[1] = inData.substring(J2start + 1, J3start).toFloat();
        JangleIn[2] = inData.substring(J3start + 1, J4start).toFloat();
        JangleIn[3] = inData.substring(J4start + 1, J5start).toFloat();
        JangleIn[4] = inData.substring(J5start + 1, J6start).toFloat();
        JangleIn[5] = inData.substring(J6start + 1, WristConStart).toFloat();
        WristCon = inData.substring(WristConStart + 1);
        WristCon.trim();

        SolveInverseKinematic();

        String echo = "";
        delay(5);
        Serial.println(inData);
      }


      //-----COMMAND TO CALIBRATE---------------------------------------------------
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
        int J1req = inData.substring(J1start + 1, J2start).toInt();
        int J2req = inData.substring(J2start + 1, J3start).toInt();
        int J3req = inData.substring(J3start + 1, J4start).toInt();
        int J4req = inData.substring(J4start + 1, J5start).toInt();
        int J5req = inData.substring(J5start + 1, J6start).toInt();
        int J6req = inData.substring(J6start + 1, J1calstart).toInt();

        float J1calOff = inData.substring(J1calstart + 1, J2calstart).toFloat();
        float J2calOff = inData.substring(J2calstart + 1, J3calstart).toFloat();
        float J3calOff = inData.substring(J3calstart + 1, J4calstart).toFloat();
        float J4calOff = inData.substring(J4calstart + 1, J5calstart).toFloat();
        float J5calOff = inData.substring(J5calstart + 1, J6calstart).toFloat();
        float J6calOff = inData.substring(J6calstart + 1).toFloat();
        ///
        float SpeedIn;
        ///
        int J1Step = 0;
        int J2Step = 0;
        int J3Step = 0;
        int J4Step = 0;
        int J5Step = 0;
        int J6Step = 0;
        ///
        int J1stepCen = 0;
        int J2stepCen = 0;
        int J3stepCen = 0;
        int J4stepCen = 0;
        int J5stepCen = 0;
        int J6stepCen = 0;
        Alarm = "0";

        //--IF JOINT IS CALLED FOR CALIBRATION PASS ITS STEP LIMIT OTHERWISE PASS 0---
        if (J1req == 1)
        {
          J1Step = J1StepLim;
        }
        if (J2req == 1)
        {
          J2Step = J2StepLim;
        }
        if (J3req == 1)
        {
          J3Step = J3StepLim;
        }
        if (J4req == 1)
        {
          J4Step = J4StepLim;
        }
        if (J5req == 1)
        {
          J5Step = J5StepLim;
        }
        if (J6req == 1)
        {
          J6Step = J6StepLim;
        }

        //--CALL FUNCT TO DRIVE TO LIMITS--
        SpeedIn = 80;
        driveLimit(J1Step, J2Step, J3Step, J4Step, J5Step, J6Step, SpeedIn);
        delay(500);

        // BACKOFF
        digitalWrite(J1dirPin, LOW);
        digitalWrite(J2dirPin, LOW);
        digitalWrite(J3dirPin, HIGH);
        digitalWrite(J4dirPin, HIGH);
        digitalWrite(J5dirPin, LOW);
        digitalWrite(J6dirPin, HIGH);

        int BacOff = 0;
        while (BacOff <= 250)
        {
          if (J1req == 1)
          {
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J1stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J2req == 1)
          {
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J2stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J3req == 1)
          {
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J3stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J4req == 1)
          {
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J4stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J5req == 1)
          {
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J5stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J6req == 1)
          {
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J6stepPin, HIGH);
            delayMicroseconds(5);
          }
          BacOff = ++BacOff;
          delayMicroseconds(4000);
        }

        //--CALL FUNCT TO DRIVE BACK TO LIMITS SLOWLY--
        SpeedIn = .02;
        driveLimit(J1Step, J2Step, J3Step, J4Step, J5Step, J6Step, SpeedIn);

        // OVERDRIVE - MAKE SURE LIMIT SWITCH STAYS MADE
        digitalWrite(J1dirPin, HIGH);
        digitalWrite(J2dirPin, HIGH);
        digitalWrite(J3dirPin, LOW);
        digitalWrite(J4dirPin, LOW);
        digitalWrite(J5dirPin, HIGH);
        digitalWrite(J6dirPin, LOW);

        int OvrDrv = 0;
        while (OvrDrv <= 30)
        {
          if (J1req == 1)
          {
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J1stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J2req == 1)
          {
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J2stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J3req == 1)
          {
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J3stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J4req == 1)
          {
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J4stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J5req == 1)
          {
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J5stepPin, HIGH);
            delayMicroseconds(5);
          }
          if (J6req == 1)
          {
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(5);
            digitalWrite(J6stepPin, HIGH);
            delayMicroseconds(5);
          }
          OvrDrv = ++OvrDrv;
          delayMicroseconds(3000);
        }

        // SEE IF ANY SWITCHES NOT MADE
        delay(500);
        ///
        if (J1req == 1)
        {
          if (digitalRead(J1calPin) == LOW)
          {
            Alarm = "1";
          }
        }
        if (J2req == 1)
        {
          if (digitalRead(J2calPin) == LOW)
          {
            Alarm = "2";
          }
        }
        if (J3req == 1)
        {
          if (digitalRead(J3calPin) == LOW)
          {
            Alarm = "3";
          }
        }
        if (J4req == 1)
        {
          if (digitalRead(J4calPin) == LOW)
          {
            Alarm = "4";
          }
        }
        if (J5req == 1)
        {
          if (digitalRead(J5calPin) == LOW)
          {
            Alarm = "5";
          }
        }
        if (J6req == 1)
        {
          if (digitalRead(J6calPin) == LOW)
          {
            Alarm = "6";
          }
        }
        ///
        if (Alarm == "0")
        {

          // set master steps and center step
          if (J1req == 1)
          {
            J1StepM = ((J1axisLim) + J1calBaseOff + J1calOff) * J1StepDeg;
            J1stepCen = ((J1axisLimPos) + J1calBaseOff + J1calOff) * J1StepDeg;
          }
          if (J2req == 1)
          {
            J2StepM = (0 + J2calBaseOff + J2calOff) * J2StepDeg;
            J2stepCen = ((J2axisLimNeg)-J2calBaseOff - J2calOff) * J2StepDeg;
          }
          if (J3req == 1)
          {
            J3StepM = ((J3axisLim) + J3calBaseOff + J3calOff) * J3StepDeg;
            J3stepCen = ((J3axisLimPos) + J3calBaseOff + J3calOff) * J3StepDeg;
          }
          if (J4req == 1)
          {
            J4StepM = (0 + J4calBaseOff + J4calOff) * J4StepDeg;
            J4stepCen = ((J4axisLimNeg)-J4calBaseOff - J4calOff) * J4StepDeg;
          }
          if (J5req == 1)
          {
            J5StepM = (0 + J5calBaseOff + J5calOff) * J5StepDeg;
            J5stepCen = ((J5axisLimNeg)-J5calBaseOff - J5calOff) * J5StepDeg;
          }
          if (J6req == 1)
          {
            J6StepM = ((J6axisLim) + J6calBaseOff + J6calOff) * J6StepDeg;
            J6stepCen = ((J6axisLimNeg) + J6calBaseOff + J6calOff) * J6StepDeg;
          }
          // move to center
          int J1dir = 0;
          int J2dir = 1;
          int J3dir = 0;
          int J4dir = 1;
          int J5dir = 1;
          int J6dir = 0;
          int TRdir = 0;
          int TRstep = 0;
          float ACCspd = 10;
          float DCCspd = 10;
          String SpeedType = "p";
          float SpeedVal = 80;
          float ACCramp = 50;

          driveMotorsJ(J1stepCen, J2stepCen, J3stepCen, J4stepCen, J5stepCen, J6stepCen, TRstep, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, TRdir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          sendRobotPos();
        }
        else
        {
          delay(5);
          Serial.println(Alarm);
        }

        inData = ""; // Clear recieved buffer
      }