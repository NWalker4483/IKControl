

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