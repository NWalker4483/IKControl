

#include <avr/pgmspace.h>

#define ROBOT_nDOFs 6
typedef float tRobotJoints[ROBOT_nDOFs];
typedef float tRobotPose[6];

//declare in out vars
float xyzuvw_Out[6];
float xyzuvw_In[7];

float JangleOut[ROBOT_nDOFs];
float JangleIn[ROBOT_nDOFs];
float joints_estimate[ROBOT_nDOFs];
float SolutionMatrix[ROBOT_nDOFs][4];

#define Table_Size 6
typedef float Matrix4x4[16];
typedef float tRobot[66];

float pose[16];

//DENAVIT HARTENBERG PARAMETERS SAME AS ROBODK

float DHparams[6][4] = {
  {    0,      0,  169.77,      0  },
  {  -90,    -90,       0,   64.2  },
  {    0,      0,       0,    305  },
  {    0,    -90,  222.63,      0  },
  {    0,     90,       0,      0  },
  {  180,    -90,   36.25,      0  }
};



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRIX OPERATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This allow to return a array as an argument instead of using global pointer

#define Matrix_Multiply(out,inA,inB)\
  (out)[0] = (inA)[0]*(inB)[0] + (inA)[4]*(inB)[1] + (inA)[8]*(inB)[2];\
  (out)[1] = (inA)[1]*(inB)[0] + (inA)[5]*(inB)[1] + (inA)[9]*(inB)[2];\
  (out)[2] = (inA)[2]*(inB)[0] + (inA)[6]*(inB)[1] + (inA)[10]*(inB)[2];\
  (out)[3] = 0;\
  (out)[4] = (inA)[0]*(inB)[4] + (inA)[4]*(inB)[5] + (inA)[8]*(inB)[6];\
  (out)[5] = (inA)[1]*(inB)[4] + (inA)[5]*(inB)[5] + (inA)[9]*(inB)[6];\
  (out)[6] = (inA)[2]*(inB)[4] + (inA)[6]*(inB)[5] + (inA)[10]*(inB)[6];\
  (out)[7] = 0;\
  (out)[8] = (inA)[0]*(inB)[8] + (inA)[4]*(inB)[9] + (inA)[8]*(inB)[10];\
  (out)[9] = (inA)[1]*(inB)[8] + (inA)[5]*(inB)[9] + (inA)[9]*(inB)[10];\
  (out)[10] = (inA)[2]*(inB)[8] + (inA)[6]*(inB)[9] + (inA)[10]*(inB)[10];\
  (out)[11] = 0;\
  (out)[12] = (inA)[0]*(inB)[12] + (inA)[4]*(inB)[13] + (inA)[8]*(inB)[14] + (inA)[12];\
  (out)[13] = (inA)[1]*(inB)[12] + (inA)[5]*(inB)[13] + (inA)[9]*(inB)[14] + (inA)[13];\
  (out)[14] = (inA)[2]*(inB)[12] + (inA)[6]*(inB)[13] + (inA)[10]*(inB)[14] + (inA)[14];\
  (out)[15] = 1;

#define Matrix_Inv(out,in)\
  (out)[0] = (in)[0];\
  (out)[1] = (in)[4];\
  (out)[2] = (in)[8];\
  (out)[3] = 0;\
  (out)[4] = (in)[1];\
  (out)[5] = (in)[5];\
  (out)[6] = (in)[9];\
  (out)[7] = 0;\
  (out)[8] = (in)[2];\
  (out)[9] = (in)[6];\
  (out)[10] = (in)[10];\
  (out)[11] = 0;\
  (out)[12] = -((in)[0]*(in)[12] + (in)[1]*(in)[13] + (in)[2]*(in)[14]);\
  (out)[13] = -((in)[4]*(in)[12] + (in)[5]*(in)[13] + (in)[6]*(in)[14]);\
  (out)[14] = -((in)[8]*(in)[12] + (in)[9]*(in)[13] + (in)[10]*(in)[14]);\
  (out)[15] = 1;

#define Matrix_Copy(out,in)\
  (out)[0]=(in)[0];\
  (out)[1]=(in)[1];\
  (out)[2]=(in)[2];\
  (out)[3]=(in)[3];\
  (out)[4]=(in)[4];\
  (out)[5]=(in)[5];\
  (out)[6]=(in)[6];\
  (out)[7]=(in)[7];\
  (out)[8]=(in)[8];\
  (out)[9]=(in)[9];\
  (out)[10]=(in)[10];\
  (out)[11]=(in)[11];\
  (out)[12]=(in)[12];\
  (out)[13]=(in)[13];\
  (out)[14]=(in)[14];\
  (out)[15]=(in)[15];

#define Matrix_Eye(inout)\
  (inout)[0] = 1;\
  (inout)[1] = 0;\
  (inout)[2] = 0;\
  (inout)[3] = 0;\
  (inout)[4] = 0;\
  (inout)[5] = 1;\
  (inout)[6] = 0;\
  (inout)[7] = 0;\
  (inout)[8] = 0;\
  (inout)[9] = 0;\
  (inout)[10] = 1;\
  (inout)[11] = 0;\
  (inout)[12] = 0;\
  (inout)[13] = 0;\
  (inout)[14] = 0;\
  (inout)[15] = 1;

#define Matrix_Multiply_Cumul(inout,inB){\
    Matrix4x4 out;\
    Matrix_Multiply(out,inout,inB);\
    Matrix_Copy(inout,out);}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//NEW DECLARATION OF VARIABLES ADDED BY OLIVIER ALLARD
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/// DHM Table parameters
#define DHM_Alpha 0
#define DHM_A 1
#define DHM_Theta 2
#define DHM_D 3


/// Custom robot base (user frame)
Matrix4x4 Robot_BaseFrame = {1, 0, 0, 0 , 0, 1, 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1};

/// Custom robot tool (tool frame, end of arm tool or TCP)
Matrix4x4 Robot_ToolFrame = {1, 0, 0, 0 , 0, 1, 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1};

/// Robot parameters
/// All robot data is held in a large array
tRobot Robot_Data = {0};


//These global variable are also pointers, allowing to put the variables inside the Robot_Data
/// DHM table
float *Robot_Kin_DHM_Table = Robot_Data + 0 * Table_Size;

/// xyzwpr of the base
float *Robot_Kin_Base = Robot_Data + 6 * Table_Size;

/// xyzwpr of the tool
float *Robot_Kin_Tool = Robot_Data + 7 * Table_Size;

/// Robot lower limits
float *Robot_JointLimits_Upper = Robot_Data + 8 * Table_Size;

/// Robot upper limits
float *Robot_JointLimits_Lower = Robot_Data + 9 * Table_Size;

/// Robot axis senses
float *Robot_Senses = Robot_Data + 10 * Table_Size;

// A value mappings

float *Robot_Kin_DHM_L1 = Robot_Kin_DHM_Table + 0 * Table_Size;
float *Robot_Kin_DHM_L2 = Robot_Kin_DHM_Table + 1 * Table_Size;
float *Robot_Kin_DHM_L3 = Robot_Kin_DHM_Table + 2 * Table_Size;
float *Robot_Kin_DHM_L4 = Robot_Kin_DHM_Table + 3 * Table_Size;
float *Robot_Kin_DHM_L5 = Robot_Kin_DHM_Table + 4 * Table_Size;
float *Robot_Kin_DHM_L6 = Robot_Kin_DHM_Table + 5 * Table_Size;


float &Robot_Kin_DHM_A2(Robot_Kin_DHM_Table[1 * Table_Size + 1]);
float &Robot_Kin_DHM_A3(Robot_Kin_DHM_Table[2 * Table_Size + 1]);
float &Robot_Kin_DHM_A4(Robot_Kin_DHM_Table[3 * Table_Size + 1]);

// D value mappings
float &Robot_Kin_DHM_D1(Robot_Kin_DHM_Table[0 * Table_Size + 3]);
float &Robot_Kin_DHM_D2(Robot_Kin_DHM_Table[1 * Table_Size + 3]);
float &Robot_Kin_DHM_D4(Robot_Kin_DHM_Table[3 * Table_Size + 3]);
float &Robot_Kin_DHM_D6(Robot_Kin_DHM_Table[5 * Table_Size + 3]);

// Theta value mappings (mastering)
float &Robot_Kin_DHM_Theta1(Robot_Kin_DHM_Table[0 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta2(Robot_Kin_DHM_Table[1 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta3(Robot_Kin_DHM_Table[2 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta4(Robot_Kin_DHM_Table[3 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta5(Robot_Kin_DHM_Table[4 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta6(Robot_Kin_DHM_Table[5 * Table_Size + 2]);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This function set the variable inside Robot_Data to the DHparams
void robot_set_AR3() {
  robot_data_reset();

  // Alpha parameters
  Robot_Kin_DHM_L1[DHM_Alpha] = DHparams[0][1] * M_PI / 180;
  Robot_Kin_DHM_L2[DHM_Alpha] = DHparams[1][1] * M_PI / 180;
  Robot_Kin_DHM_L3[DHM_Alpha] = DHparams[2][1] * M_PI / 180;
  Robot_Kin_DHM_L4[DHM_Alpha] = DHparams[3][1] * M_PI / 180;
  Robot_Kin_DHM_L5[DHM_Alpha] = DHparams[4][1] * M_PI / 180;
  Robot_Kin_DHM_L6[DHM_Alpha] = DHparams[5][1] * M_PI / 180;

  // Theta parameters
  Robot_Kin_DHM_L1[DHM_Theta] = DHparams[0][0] * M_PI / 180;
  Robot_Kin_DHM_L2[DHM_Theta] = DHparams[1][0] * M_PI / 180;
  Robot_Kin_DHM_L3[DHM_Theta] = DHparams[2][0] * M_PI / 180;
  Robot_Kin_DHM_L4[DHM_Theta] = DHparams[3][0] * M_PI / 180;
  Robot_Kin_DHM_L5[DHM_Theta] = DHparams[4][0] * M_PI / 180;
  Robot_Kin_DHM_L6[DHM_Theta] = DHparams[5][0] * M_PI / 180;

  // A parameters
  Robot_Kin_DHM_L1[DHM_A] = DHparams[0][3];
  Robot_Kin_DHM_L2[DHM_A] = DHparams[1][3];
  Robot_Kin_DHM_L3[DHM_A] = DHparams[2][3];
  Robot_Kin_DHM_L4[DHM_A] = DHparams[3][3];
  Robot_Kin_DHM_L5[DHM_A] = DHparams[4][3];
  Robot_Kin_DHM_L6[DHM_A] = DHparams[5][3];

  // D parameters
  Robot_Kin_DHM_L1[DHM_D] = DHparams[0][2];
  Robot_Kin_DHM_L2[DHM_D] = DHparams[1][2];
  Robot_Kin_DHM_L3[DHM_D] = DHparams[2][2];
  Robot_Kin_DHM_L4[DHM_D] = DHparams[3][2];
  Robot_Kin_DHM_L5[DHM_D] = DHparams[4][2];
  Robot_Kin_DHM_L6[DHM_D] = DHparams[5][2];


  Robot_JointLimits_Lower[0] = J1axisLimNeg;
  Robot_JointLimits_Upper[0] = J1axisLimPos;
  Robot_JointLimits_Lower[1] = J2axisLimNeg;
  Robot_JointLimits_Upper[1] = J2axisLimPos;
  Robot_JointLimits_Lower[2] = J3axisLimNeg;
  Robot_JointLimits_Upper[2] = J3axisLimPos;
  Robot_JointLimits_Lower[3] = J4axisLimNeg;
  Robot_JointLimits_Upper[3] = J4axisLimPos;
  Robot_JointLimits_Lower[4] = J5axisLimNeg;
  Robot_JointLimits_Upper[4] = J5axisLimPos;
  Robot_JointLimits_Lower[5] = J6axisLimNeg;
  Robot_JointLimits_Upper[5] = J6axisLimPos;

}

void robot_data_reset() {
  // Reset user base and tool frames
  Matrix_Eye(Robot_BaseFrame);
  Matrix_Eye(Robot_ToolFrame);

  // Reset internal base frame and tool frames
  for (int i = 0; i < 6; i++) {
    Robot_Kin_Base[i] = 0.0;
  }

  // Reset joint senses and joint limits
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Robot_Senses[i] = +1.0;

  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRICE OPERATIONS ADDED BY OLIVIER ALLARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
bool robot_joints_valid(const T joints[ROBOT_nDOFs]) {

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    if (joints[i] < -Robot_JointLimits_Lower[i] || joints[i] > Robot_JointLimits_Upper[i]) {
      return false;
    }
  }
  return true;
}


//This function return a 4x4 matrix as an argument (pose) following the modified DH rules for the inputs T rx, T tx, T rz and T tz source : https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
template <typename T>
void DHM_2_pose(T rx, T tx, T rz, T tz, Matrix4x4 pose)
{
  T crx;
  T srx;
  T crz;
  T srz;
  crx = cos(rx);
  srx = sin(rx);
  crz = cos(rz);
  srz = sin(rz);
  pose[0] = crz;
  pose[4] = -srz;
  pose[8] = 0.0;
  pose[12] = tx;
  pose[1] = crx * srz;
  pose[5] = crx * crz;
  pose[9] = -srx;
  pose[13] = -tz * srx;
  pose[2] = srx * srz;
  pose[6] = crz * srx;
  pose[10] = crx;
  pose[14] = tz * crx;
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


//This function tranform a coordinate system xyzwpr into a 4x4 matrix and return it as an argument.
template <typename T>
void xyzwpr_2_pose(const T xyzwpr[6], Matrix4x4 pose)
{
  T srx;
  T crx;
  T sry;
  T cry;
  T srz;
  T crz;
  T H_tmp;
  srx = sin(xyzwpr[3]);
  crx = cos(xyzwpr[3]);
  sry = sin(xyzwpr[4]);
  cry = cos(xyzwpr[4]);
  srz = sin(xyzwpr[5]);
  crz = cos(xyzwpr[5]);
  pose[0] = cry * crz;
  pose[4] = -cry * srz;
  pose[8] = sry;
  pose[12] = xyzwpr[0];
  H_tmp = crz * srx;
  pose[1] = crx * srz + H_tmp * sry;
  crz *= crx;
  pose[5] = crz - srx * sry * srz;
  pose[9] = -cry * srx;
  pose[13] = xyzwpr[1];
  pose[2] = srx * srz - crz * sry;
  pose[6] = H_tmp + crx * sry * srz;
  pose[10] = crx * cry;
  pose[14] = xyzwpr[2];
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


/// Calculate the [x,y,z,u,v,w] position with rotation vector for a pose target
template <typename T>
void pose_2_xyzuvw(const Matrix4x4 pose, T out[6])
{
  T sin_angle;
  T angle;
  T vector[3];
  int iidx;
  int vector_tmp;
  signed char b_I[9];
  out[0] = pose[12];
  out[1] = pose[13];
  out[2] = pose[14];
  sin_angle = (((pose[0] + pose[5]) + pose[10]) - 1.0) * 0.5;
  if (sin_angle <= -1.0) {
    sin_angle = -1.0;
  }

  if (sin_angle >= 1.0) {
    sin_angle = 1.0;
  }

  angle = acos(sin_angle);
  if (angle < 1.0E-6) {
    vector[0] = 0.0;
    vector[1] = 0.0;
    vector[2] = 0.0;
  } else {
    sin_angle = sin(angle);
    if (abs(sin_angle) < 1.0E-6) { //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      sin_angle = pose[0];
      iidx = 0;
      if (pose[0] < pose[5]) {
        sin_angle = pose[5];
        iidx = 1;
      }

      if (sin_angle < pose[10]) {
        sin_angle = pose[10];
        iidx = 2;
      }

      for (vector_tmp = 0; vector_tmp < 9; vector_tmp++) {
        b_I[vector_tmp] = 0;
      }

      b_I[0] = 1;
      b_I[4] = 1;
      b_I[8] = 1;
      sin_angle = 2.0 * (1.0 + sin_angle);
      if (sin_angle <= 0.0) {
        sin_angle = 0.0;
      } else {
        sin_angle = sqrt(sin_angle);
      }

      vector_tmp = iidx << 2;
      vector[0] = (pose[vector_tmp] + static_cast<T>(b_I[3 * iidx])) /
                  sin_angle;
      vector[1] = (pose[1 + vector_tmp] + static_cast<T>(b_I[1 + 3 * iidx]))
                  / sin_angle;
      vector[2] = (pose[2 + vector_tmp] + static_cast<T>(b_I[2 + 3 * iidx]))
                  / sin_angle;
      angle = M_PI;
    } else {
      sin_angle = 1.0 / (2.0 * sin_angle);
      vector[0] = (pose[6] - pose[9]) * sin_angle;
      vector[1] = (pose[8] - pose[2]) * sin_angle;
      vector[2] = (pose[1] - pose[4]) * sin_angle;
    }
  }

  sin_angle = angle * 180.0 / M_PI;
  out[3] = vector[0] * sin_angle * M_PI / 180.0;
  out[4] = vector[1] * sin_angle * M_PI / 180.0;
  out[5] = vector[2] * sin_angle * M_PI / 180.0;
}


//This function tranform a coordinate system xyzwpr into a 4x4 matrix using UR euler rules and return it as an argument.
template <typename T>
void xyzuvw_2_pose(const T xyzuvw[6], Matrix4x4 pose)
{
  T s;
  T angle;
  T axisunit[3];
  T ex;
  T c;
  T pose_tmp;
  T b_pose_tmp;
  s = sqrt((xyzuvw[3] * xyzuvw[3] + xyzuvw[4] * xyzuvw[4]) + xyzuvw[5] *
           xyzuvw[5]);
  angle = s * 180.0 / M_PI;
  if (abs(angle) < 1.0E-6) { //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
    memset(&pose[0], 0, sizeof(T) << 4);
    pose[0] = 1.0;
    pose[5] = 1.0;
    pose[10] = 1.0;
    pose[15] = 1.0;
  } else {
    axisunit[1] = abs(xyzuvw[4]);
    axisunit[2] = abs(xyzuvw[5]);
    ex = abs(xyzuvw[3]);
    if (abs(xyzuvw[3]) < axisunit[1]) {
      ex = axisunit[1];
    }

    if (ex < axisunit[2]) {
      ex = axisunit[2];
    }

    if (ex < 1.0E-6) { //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      memset(&pose[0], 0, sizeof(T) << 4);
      pose[0] = 1.0;
      pose[5] = 1.0;
      pose[10] = 1.0;
      pose[15] = 1.0;
    } else {
      axisunit[0] = xyzuvw[3] / s;
      axisunit[1] = xyzuvw[4] / s;
      axisunit[2] = xyzuvw[5] / s;
      s = angle * 3.1415926535897931 / 180.0;
      c = cos(s);
      s = sin(s);
      angle = axisunit[0] * axisunit[0];
      pose[0] = angle + c * (1.0 - angle);
      angle = axisunit[0] * axisunit[1] * (1.0 - c);
      ex = axisunit[2] * s;
      pose[4] = angle - ex;
      pose_tmp = axisunit[0] * axisunit[2] * (1.0 - c);
      b_pose_tmp = axisunit[1] * s;
      pose[8] = pose_tmp + b_pose_tmp;
      pose[1] = angle + ex;
      angle = axisunit[1] * axisunit[1];
      pose[5] = angle + (1.0 - angle) * c;
      angle = axisunit[1] * axisunit[2] * (1.0 - c);
      ex = axisunit[0] * s;
      pose[9] = angle - ex;
      pose[2] = pose_tmp - b_pose_tmp;
      pose[6] = angle + ex;
      angle = axisunit[2] * axisunit[2];
      pose[10] = angle + (1.0 - angle) * c;
      pose[3] = 0.0;
      pose[7] = 0.0;
      pose[11] = 0.0;
      pose[15] = 1.0;
    }
  }

  pose[12] = xyzuvw[0];
  pose[13] = xyzuvw[1];
  pose[14] = xyzuvw[2];
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FOWARD KINEMATIC ADDED BY OLIVIER ALLARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This function input the JxangleIn into an array, send it to the foward kinematic solver and output the result into the position variables
void SolveFowardKinematic() {

  robot_set_AR3();

  float target_xyzuvw[6];
  float joints[ROBOT_nDOFs];

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints[i] = JangleIn[i];
  }


  forward_kinematics_robot_xyzuvw(joints, target_xyzuvw);

  xyzuvw_Out[0] = target_xyzuvw[0];
  xyzuvw_Out[1] = target_xyzuvw[1];
  xyzuvw_Out[2] = target_xyzuvw[2];
  xyzuvw_Out[3] = target_xyzuvw[3] / M_PI * 180;
  xyzuvw_Out[4] = target_xyzuvw[4] / M_PI * 180;
  xyzuvw_Out[5] = target_xyzuvw[5] / M_PI * 180;

}



template <typename T>
void forward_kinematics_arm(const T *joints, Matrix4x4 pose) {
  xyzwpr_2_pose(Robot_Kin_Base, pose);
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Matrix4x4 hi;
    float *dhm_i = Robot_Kin_DHM_Table + i * Table_Size;
    T ji_rad = joints[i] * Robot_Senses[i] * M_PI / 180.0;
    DHM_2_pose(dhm_i[0], dhm_i[1], dhm_i[2] + ji_rad, dhm_i[3], hi);
    Matrix_Multiply_Cumul(pose, hi);
  }
  Matrix4x4 tool_pose;
  xyzwpr_2_pose(Robot_Kin_Tool, tool_pose);
  Matrix_Multiply_Cumul(pose, tool_pose);
}

template <typename T>
void forward_kinematics_robot_xyzuvw(const T joints[ROBOT_nDOFs], T target_xyzuvw[6]) {
  Matrix4x4 pose;
  forward_kinematics_robot(joints, pose);    //send the joints values and return the pose matrix as an argument
  pose_2_xyzuvw(pose, target_xyzuvw);        //send the pose matrix and return the xyzuvw values in an array as an argument
}

//Calculate de foward kinematic of the robot without the tool
template <typename T>
void forward_kinematics_robot(const T joints[ROBOT_nDOFs], Matrix4x4 target) {
  Matrix4x4 invBaseFrame;
  Matrix4x4 pose_arm;
  Matrix_Inv(invBaseFrame, Robot_BaseFrame); // invRobot_Tool could be precalculated, the tool does not change so often
  forward_kinematics_arm(joints, pose_arm);
  Matrix_Multiply(target, invBaseFrame, pose_arm);
  Matrix_Multiply_Cumul(target, Robot_ToolFrame);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//REVERSE KINEMATIC ADDED BY OLIVIER ALLARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updatejoints() {

  for (int i = 0; i > ROBOT_nDOFs; i++) {
    JangleIn[i] = JangleOut[i];
  }

}

void JointEstimate() {

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints_estimate[i] = JangleIn[i];
  }
}

void SolveInverseKinematic() {

  float joints[ROBOT_nDOFs];
  float target[6];

  float solbuffer[ROBOT_nDOFs] = {0};
  int NumberOfSol = 0;
  int solVal = 0;

  KinematicError = 0;

  JointEstimate();
  target[0] = xyzuvw_In[0];
  target[1] = xyzuvw_In[1];
  target[2] = xyzuvw_In[2];
  target[3] = xyzuvw_In[3] * M_PI / 180;
  target[4] = xyzuvw_In[4] * M_PI / 180;
  target[5] = xyzuvw_In[5] * M_PI / 180;

  for (int i = -3; i <= 3; i++) {
    joints_estimate[4] = i * 30;
    int success = inverse_kinematics_robot_xyzuvw<float>(target, joints, joints_estimate);
    if (success) {
      if (solbuffer[4] != joints[4]) {
        if (robot_joints_valid(joints)) {
          for (int j = 0; j < ROBOT_nDOFs; j++) {
            solbuffer[j] = joints[j];
            SolutionMatrix[j][NumberOfSol] = solbuffer[j];
          }
          if (NumberOfSol <= 6) {
            NumberOfSol++;
          }
        }
      }
    } else {
      KinematicError = 1;
    }
  }

  joints_estimate[4] = JangleIn[4];


  solVal = 0;
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    if ((abs(joints_estimate[i] - SolutionMatrix[i][0]) > 20) and NumberOfSol > 1) {
      solVal = 1;
    } else if ((abs(joints_estimate[i] - SolutionMatrix[i][1]) > 20) and NumberOfSol > 1) {
      solVal = 0;
    }


   // Serial.println(String(i) + "  Joint estimate : " + String(joints_estimate[i]) + " // Joint sol 1 : " + String(SolutionMatrix[i][0]) + " // Joint sol 2 : " + String(SolutionMatrix[i][1]));
  }

  if (NumberOfSol==0) {
    KinematicError = 1;
  }


 // Serial.println("Sol : " + String(solVal) + " Nb sol : " + String(NumberOfSol));

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    JangleOut[i] = SolutionMatrix[i][solVal];
  }

}





template <typename T>
int inverse_kinematics_robot(const Matrix4x4 target, T joints[ROBOT_nDOFs], const T *joints_estimate) {
  Matrix4x4 invToolFrame;
  Matrix4x4 pose_arm;
  int nsol;
  Matrix_Inv(invToolFrame, Robot_ToolFrame); // invRobot_Tool could be precalculated, the tool does not change so often
  Matrix_Multiply(pose_arm, Robot_BaseFrame, target);
  Matrix_Multiply_Cumul(pose_arm, invToolFrame);
  if (joints_estimate != nullptr) {
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_estimate, joints, &nsol);
  } else {
    // Warning! This is dangerous if joints does not have a valid/reasonable result
    T joints_approx[6];
    memcpy(joints_approx, joints, ROBOT_nDOFs * sizeof(T));
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_approx, joints, &nsol);
  }
  if (nsol == 0) {
    return 0;
  }

  return 1;
}


template <typename T>
int inverse_kinematics_robot_xyzuvw(const T target_xyzuvw1[6], T joints[ROBOT_nDOFs], const T *joints_estimate) {

  Matrix4x4 pose;
  xyzuvw_2_pose(target_xyzuvw1, pose);
  return inverse_kinematics_robot(pose, joints, joints_estimate);
}


template <typename T>
void inverse_kinematics_raw(const T pose[16], const tRobot DK, const T joints_approx_in[6], T joints[6], int *nsol)
{
  int i0;
  T base[16];
  T joints_approx[6];
  T tool[16];
  int i;
  T Hout[16];
  T b_Hout[9];
  T dv0[4];
  bool guard1 = false;
  T make_sqrt;
  T P04[4];
  T q1;
  int i1;
  T c_Hout[16];
  T k2;
  T k1;
  T ai;
  T B;
  T C;
  T s31;
  T c31;
  T q13_idx_2;
  T bb_div_cc;
  T q13_idx_0;
  for (i0 = 0; i0 < 6; i0++) {
    joints_approx[i0] = DK[60 + i0] * joints_approx_in[i0];
  }

  xyzwpr_2_pose(*(T (*)[6])&DK[36], base);
  xyzwpr_2_pose(*(T (*)[6])&DK[42], tool);
  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    Hout[i] = base[i0];
    Hout[1 + i] = base[i0 + 4];
    Hout[2 + i] = base[i0 + 8];
    Hout[3 + i] = base[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    Hout[3 + i] = 0.0;
    b_Hout[3 * i0] = -Hout[i];
    b_Hout[1 + 3 * i0] = -Hout[1 + i];
    b_Hout[2 + 3 * i0] = -Hout[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    Hout[12 + i0] = (b_Hout[i0] * base[12] + b_Hout[i0 + 3] * base[13]) + b_Hout[i0 + 6] * base[14];
  }

  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    base[i] = tool[i0];
    base[1 + i] = tool[i0 + 4];
    base[2 + i] = tool[i0 + 8];
    base[3 + i] = tool[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    base[3 + i] = 0.0;
    b_Hout[3 * i0] = -base[i];
    b_Hout[1 + 3 * i0] = -base[1 + i];
    b_Hout[2 + 3 * i0] = -base[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    base[12 + i0] = (b_Hout[i0] * tool[12] + b_Hout[i0 + 3] * tool[13]) + b_Hout[i0 + 6] * tool[14];
  }

  dv0[0] = 0.0;
  dv0[1] = 0.0;
  dv0[2] = -DK[33];
  dv0[3] = 1.0;
  for (i0 = 0; i0 < 4; i0++) {
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      c_Hout[i0 + i1] = ((Hout[i0] * pose[i1] + Hout[i0 + 4] * pose[1 + i1]) + Hout[i0 + 8] * pose[2 + i1]) + Hout[i0 + 12] * pose[3 + i1];
    }

    P04[i0] = 0.0;
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      make_sqrt = ((c_Hout[i0] * base[i1] + c_Hout[i0 + 4] * base[1 + i1]) + c_Hout[i0 + 8] * base[2 + i1]) + c_Hout[i0 + 12] * base[3 + i1];
      tool[i0 + i1] = make_sqrt;
      P04[i0] += make_sqrt * dv0[i];
    }
  }

  guard1 = false;
  if (DK[9] == 0.0) {
    q1 = atan2(P04[1], P04[0]);
    guard1 = true;
  } else {
    make_sqrt = (P04[0] * P04[0] + P04[1] * P04[1]) - DK[9] * DK[9];
    if (make_sqrt < 0.0) {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    } else {
      q1 = atan2(P04[1], P04[0]) - atan2(DK[9], sqrt(make_sqrt));
      guard1 = true;
    }
  }

  if (guard1) {
    k2 = P04[2] - DK[3];
    k1 = (cos(q1) * P04[0] + sin(q1) * P04[1]) - DK[7];
    ai = (((k1 * k1 + k2 * k2) - DK[13] * DK[13]) - DK[21] * DK[21]) - DK[19] * DK[19];
    B = 2.0 * DK[21] * DK[13];
    C = 2.0 * DK[19] * DK[13];
    s31 = 0.0;
    c31 = 0.0;
    if (C == 0.0) {
      s31 = -ai / B;
      make_sqrt = 1.0 - s31 * s31;
      if (make_sqrt >= 0.0) {
        c31 = sqrt(make_sqrt);
      }
    } else {
      q13_idx_2 = C * C;
      bb_div_cc = B * B / q13_idx_2;
      make_sqrt = 2.0 * ai * B / q13_idx_2;
      make_sqrt = make_sqrt * make_sqrt - 4.0 * ((1.0 + bb_div_cc) * (ai * ai / q13_idx_2 - 1.0));
      if (make_sqrt >= 0.0) {
        s31 = (-2.0 * ai * B / q13_idx_2 + sqrt(make_sqrt)) / (2.0 * (1.0 + bb_div_cc));
        c31 = (ai + B * s31) / C;
      }
    }

    if ((make_sqrt >= 0.0) && (abs(s31) <= 1.0)) {
      B = atan2(s31, c31);
      make_sqrt = cos(B);
      ai = sin(B);
      C = (DK[13] - DK[21] * ai) + DK[19] * make_sqrt;
      make_sqrt = DK[21] * make_sqrt + DK[19] * ai;
      q13_idx_0 = q1 + -DK[2];
      k2 = atan2(C * k1 - make_sqrt * k2, C * k2 + make_sqrt * k1) + (-DK[8] - M_PI / 2);
      q13_idx_2 = B + -DK[14];
      bb_div_cc = joints_approx[3] * M_PI / 180.0 - (-DK[20]);
      q1 = q13_idx_0 + DK[2];
      B = k2 + DK[8];
      C = q13_idx_2 + DK[14];
      make_sqrt = B + C;
      s31 = cos(make_sqrt);
      c31 = cos(q1);
      Hout[0] = s31 * c31;
      ai = sin(q1);
      Hout[4] = s31 * ai;
      make_sqrt = sin(make_sqrt);
      Hout[8] = -make_sqrt;
      Hout[12] = (DK[3] * make_sqrt - DK[7] * s31) - DK[13] * cos(C);
      Hout[1] = -sin(B + C) * c31;
      Hout[5] = -sin(B + C) * ai;
      Hout[9] = -s31;
      Hout[13] = (DK[3] * s31 + DK[7] * make_sqrt) + DK[13] * sin(C);
      Hout[2] = -ai;
      Hout[6] = c31;
      Hout[10] = 0.0;
      Hout[14] = 0.0;
      Hout[3] = 0.0;
      Hout[7] = 0.0;
      Hout[11] = 0.0;
      Hout[15] = 1.0;
      for (i0 = 0; i0 < 4; i0++) {
        for (i = 0; i < 4; i++) {
          i1 = i << 2;
          base[i0 + i1] = ((Hout[i0] * tool[i1] + Hout[i0 + 4] * tool[1 + i1]) + Hout[i0 + 8] * tool[2 + i1]) + Hout[i0 + 12] * tool[3 + i1];
        }
      }

      make_sqrt = 1.0 - base[9] * base[9];
      if (make_sqrt <= 0.0) {
        make_sqrt = 0.0;
      } else {
        make_sqrt = sqrt(make_sqrt);
      }

      if (make_sqrt < 1.0E-6) {
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(bb_div_cc);
        ai = cos(bb_div_cc);
        make_sqrt = atan2(make_sqrt * base[0] + ai * base[2], make_sqrt * base[2] - ai * base[0]);
      } else if (joints_approx[4] >= 0.0) {
        bb_div_cc = atan2(base[10] / make_sqrt, -base[8] / make_sqrt);
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      } else {
        bb_div_cc = atan2(-base[10] / make_sqrt, base[8] / make_sqrt);
        C = atan2(-make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      }

      joints[0] = q13_idx_0;
      joints[3] = bb_div_cc + -DK[20];
      joints[1] = k2;
      joints[4] = C + -DK[26];
      joints[2] = q13_idx_2;
      joints[5] = make_sqrt + (-DK[32] + M_PI);
      make_sqrt = joints[5];
      if (joints[5] > 3.1415926535897931) {
        make_sqrt = joints[5] - M_PI * 2;
      } else {
        if (joints[5] <= -M_PI) {
          make_sqrt = joints[5] + M_PI * 2;
        }
      }

      joints[5] = make_sqrt;
      for (i0 = 0; i0 < 6; i0++) {
        joints[i0] = DK[60 + i0] * (joints[i0] * 180.0 / M_PI);
      }

      *nsol = 1.0;
    } else {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    }
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CALCULATE POSITIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendRobotPos() {

  updatePos();
  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + TRStepM;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";
}

void updatePos() {

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

  SolveFowardKinematic();
}



void correctRobotPos () {

  J1StepM = J1encPos.read() / J1encMult;
  J2StepM = J2encPos.read() / J2encMult;
  J3StepM = J3encPos.read() / J3encMult;
  J4StepM = J4encPos.read() / J4encMult;
  J5StepM = J5encPos.read() / J5encMult;
  J6StepM = J6encPos.read() / J6encMult;

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;


  SolveFowardKinematic();

  //String sendPos = "A" + String(J1StepM) + "B" + String(J2StepM) + "C" + String(J3StepM) + "D" + String(J4StepM) + "E" + String(J5StepM) + "F" + String(J6StepM) + "G" + String(xyzuvw_Out[0], 2) + "H" + String(xyzuvw_Out[1], 2) + "I" + String(xyzuvw_Out[2], 2) + "J" + String(xyzuvw_Out[3], 2) + "K" + String(xyzuvw_Out[4], 2) + "L" + String(xyzuvw_Out[5], 2) + "M" + speedViolation + "N" + debug + "O"+ flag + "P" + TRStepM;
  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + TRStepM;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";



}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE LIMIT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveLimit(int J1Step, int J2Step, int J3Step, int J4Step, int J5Step, int J6Step, float SpeedVal) {

  //RESET COUNTERS
  int J1done = 0;
  int J2done = 0;
  int J3done = 0;
  int J4done = 0;
  int J5done = 0;
  int J6done = 0;

  int J1complete = 0;
  int J2complete = 0;
  int J3complete = 0;
  int J4complete = 0;
  int J5complete = 0;
  int J6complete = 0;

  int calcStepGap = ((maxSpeedDelay - ((SpeedVal / 100) * maxSpeedDelay)) + minSpeedDelay + 300);

  //SET CAL DIRECTION
  digitalWrite(J1dirPin, HIGH);
  digitalWrite(J2dirPin, HIGH);
  digitalWrite(J3dirPin, LOW);
  digitalWrite(J4dirPin, LOW);
  digitalWrite(J5dirPin, HIGH);
  digitalWrite(J6dirPin, LOW);

  //DRIVE MOTORS FOR CALIBRATION

  int curRead;
  int J1CurState;
  int J2CurState;
  int J3CurState;
  int J4CurState;
  int J5CurState;
  int J6CurState;
  int DriveLimInProc = 1;

  if (J1Step <= 0 ) {
    J1complete = 1;
  }
  if (J2Step <= 0) {
    J2complete = 1;
  }
  if (J3Step <= 0) {
    J3complete = 1;
  }
  if (J4Step <= 0) {
    J4complete = 1;
  }
  if (J5Step <= 0) {
    J5complete = 1;
  }
  if (J6Step <= 0) {
    J6complete = 1;
  }


  while (DriveLimInProc == 1) {

    //EVAL J1
    if (digitalRead(J1calPin) == LOW) {
      J1CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J1calPin) == LOW) {
        J1CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J1calPin) == LOW) {
          J1CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J1calPin) == LOW) {
            J1CurState = LOW;
          }
          else {
            J1CurState = digitalRead(J1calPin);
          }
        }
      }
    }

    //EVAL J2
    if (digitalRead(J2calPin) == LOW) {
      J2CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J2calPin) == LOW) {
        J2CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J2calPin) == LOW) {
          J2CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J2calPin) == LOW) {
            J2CurState = LOW;
          }
          else {
            J2CurState = digitalRead(J2calPin);
          }
        }
      }
    }

    //EVAL J3
    if (digitalRead(J3calPin) == LOW) {
      J3CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J3calPin) == LOW) {
        J3CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J3calPin) == LOW) {
          J3CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J3calPin) == LOW) {
            J3CurState = LOW;
          }
          else {
            J3CurState = digitalRead(J3calPin);
          }
        }
      }
    }

    //EVAL J4
    if (digitalRead(J4calPin) == LOW) {
      J4CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J4calPin) == LOW) {
        J4CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J4calPin) == LOW) {
          J4CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J4calPin) == LOW) {
            J4CurState = LOW;
          }
          else {
            J4CurState = digitalRead(J4calPin);
          }
        }
      }
    }

    //EVAL J5
    if (digitalRead(J5calPin) == LOW) {
      J5CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J5calPin) == LOW) {
        J5CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J5calPin) == LOW) {
          J5CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J5calPin) == LOW) {
            J5CurState = LOW;
          }
          else {
            J5CurState = digitalRead(J5calPin);
          }
        }
      }
    }

    //EVAL J6
    if (digitalRead(J6calPin) == LOW) {
      J6CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J6calPin) == LOW) {
        J6CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J6calPin) == LOW) {
          J6CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J6calPin) == LOW) {
            J6CurState = LOW;
          }
          else {
            J6CurState = digitalRead(J6calPin);
          }
        }
      }
    }

    if (J1done < J1Step && J1CurState == LOW) {
      digitalWrite(J1stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J1stepPin, HIGH);
      J1done = ++J1done;
    }
    else {
      J1complete = 1;
    }
    if (J2done < J2Step && J2CurState == LOW) {
      digitalWrite(J2stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J2stepPin, HIGH);
      J2done = ++J2done;
    }
    else {
      J2complete = 1;
    }
    if (J3done < J3Step && J3CurState == LOW) {
      digitalWrite(J3stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J3stepPin, HIGH);
      J3done = ++J3done;
    }
    else {
      J3complete = 1;
    }
    if (J4done < J4Step && J4CurState == LOW) {
      digitalWrite(J4stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J4stepPin, HIGH);
      J4done = ++J4done;
    }
    else {
      J4complete = 1;
    }
    if (J5done < J5Step && J5CurState == LOW) {
      digitalWrite(J5stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J5stepPin, HIGH);
      J5done = ++J5done;
    }
    else {
      J5complete = 1;
    }
    if (J6done < J6Step && J6CurState == LOW) {
      digitalWrite(J6stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J6stepPin, HIGH);
      J6done = ++J6done;
    }
    else {
      J6complete = 1;
    }
    //jump out if complete
    if (J1complete + J2complete + J3complete + J4complete + J5complete + J6complete == 6) {
      DriveLimInProc = 0;
    }
    ///////////////DELAY BEFORE RESTARTING LOOP
    delayMicroseconds(calcStepGap);
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CHECK ENCODERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS J
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsJ(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int TRstep, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int TRdir, String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp) {

  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (TRstep > HighStep)
  {
    HighStep = TRstep;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int TRactive = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (TRstep >= 1)
  {
    TRactive = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int TR_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int TR_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int TR_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int TR_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int TR_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int TRcur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int TR_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int TR_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int TR_SE_2cur = 0;

  int highStepCur = 0;
  float curDelay = 0;

  float speedSP;
  float moveDist;

  //int J4EncSteps;

  //SET DIRECTIONS

  /// J1 ///
  if (J1dir) {
    digitalWrite(J1dirPin, HIGH);
  }
  else {
    digitalWrite(J1dirPin, LOW);
  }
  /// J2 ///
  if (J2dir) {
    digitalWrite(J2dirPin, LOW);
  }
  else {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3dir) {
    digitalWrite(J3dirPin, LOW);
  }
  else {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4dir) {
    digitalWrite(J4dirPin, HIGH);
  }
  else {
    digitalWrite(J4dirPin, LOW);
  }
  /// J5 ///
  if (J5dir) {
    digitalWrite(J5dirPin, LOW);
  }
  else {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6dir) {
    digitalWrite(J6dirPin, LOW);
  }
  else {
    digitalWrite(J6dirPin, HIGH);
  }
  /// J7 ///
  if (TRdir) {
    digitalWrite(TRdirPin, HIGH);
  }
  else {
    digitalWrite(TRdirPin, LOW);
  }

  /////CALC SPEEDS//////
  float calcStepGap;

  //determine steps
  float ACCStep = HighStep * (ACCspd / 100);
  float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
  float DCCStep = HighStep * (DCCspd / 100);

  //set speed for seconds or mm per sec
  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000) * .8;
  }
  else if (SpeedType == "m") {
    lineDist = pow((pow((xyzuvw_In[0] - xyzuvw_Out[0]), 2) + pow((xyzuvw_In[1] - xyzuvw_Out[1]), 2) + pow((xyzuvw_In[2] - xyzuvw_Out[2]), 2)), .5);
    speedSP = ((lineDist / SpeedVal) * 1000000) * .8;
  }

  //calc step gap for seconds or mm per sec
  if (SpeedType == "s" or SpeedType == "m" ) {
    float zeroStepGap = speedSP / HighStep;
    float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
    float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
    float zeroNORtime = NORStep * zeroStepGap;
    float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
    float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
    float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
    float overclockPerc = speedSP / zeroTOTtime;
    calcStepGap = zeroStepGap * overclockPerc;
    if (calcStepGap <= minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  }

  //calc step gap for percentage
  else if (SpeedType == "p") {
    calcStepGap = (maxSpeedDelay - ((SpeedVal / 100) * maxSpeedDelay));
    if (calcStepGap < minSpeedDelay) {
      calcStepGap = minSpeedDelay;
    }
  }

  //calculate final step increments
  float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
  float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
  float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;

  //set starting delay
  curDelay = calcACCstartDel;

  ///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
  {

    ////DELAY CALC/////
    if (highStepCur <= ACCStep) {
      curDelay = curDelay - (calcACCstepInc);
    }
    else if (highStepCur >= (HighStep - DCCStep)) {
      curDelay = curDelay + (calcDCCstepInc);
    }
    else {
      curDelay = calcStepGap;
    }



    float distDelay = 60;
    if (debugg == 1) {
      distDelay = 0;
    }
    float disDelayCur = 0;


    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J1dir == 0) {
              J1StepM == --J1StepM;
            }
            else {
              J1StepM == ++J1StepM;
            }
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }


    /////// J2 ////////////////////////////////
    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J2dir == 0) {
              J2StepM == --J2StepM;
            }
            else {
              J2StepM == ++J2StepM;
            }
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J3dir == 0) {
              J3StepM == --J3StepM;
            }
            else {
              J3StepM == ++J3StepM;
            }
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }


    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J4dir == 0) {
              J4StepM == --J4StepM;
            }
            else {
              J4StepM == ++J4StepM;
            }
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }


    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J5dir == 0) {
              J5StepM == --J5StepM;
            }
            else {
              J5StepM == ++J5StepM;
            }
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }


    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J6dir == 0) {
              J6StepM == --J6StepM;
            }
            else {
              J6StepM == ++J6StepM;
            }
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }


    /////// TR ////////////////////////////////
    ///find pulse every
    if (TRcur < TRstep)
    {
      TR_PE = (HighStep / TRstep);
      ///find left over 1
      TR_LO_1 = (HighStep - (TRstep * TR_PE));
      ///find skip 1
      if (TR_LO_1 > 0)
      {
        TR_SE_1 = (HighStep / TR_LO_1);
      }
      else
      {
        TR_SE_1 = 0;
      }
      ///find left over 2
      if (TR_SE_1 > 0)
      {
        TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
      }
      else
      {
        TR_LO_2 = 0;
      }
      ///find skip 2
      if (TR_LO_2 > 0)
      {
        TR_SE_2 = (HighStep / TR_LO_2);
      }
      else
      {
        TR_SE_2 = 0;
      }
      /////////  TR  ///////////////
      if (TR_SE_2 == 0)
      {
        TR_SE_2cur = (TR_SE_2 + 1);
      }
      if (TR_SE_2cur != TR_SE_2)
      {
        TR_SE_2cur = ++TR_SE_2cur;
        if (TR_SE_1 == 0)
        {
          TR_SE_1cur = (TR_SE_1 + 1);
        }
        if (TR_SE_1cur != TR_SE_1)
        {
          TR_SE_1cur = ++TR_SE_1cur;
          TR_PEcur = ++TR_PEcur;
          if (TR_PEcur == TR_PE)
          {
            TRcur = ++TRcur;
            TR_PEcur = 0;
            digitalWrite(TRstepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (TRdir == 0) {
              TRStepM == --TRStepM;
            }
            else {
              TRStepM == ++TRStepM;
            }
          }
        }
        else
        {
          TR_SE_1cur = 0;
        }
      }
      else
      {
        TR_SE_2cur = 0;
      }
    }

    // inc cur step
    highStepCur = ++highStepCur;
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    digitalWrite(J5stepPin, HIGH);
    digitalWrite(J6stepPin, HIGH);
    digitalWrite(TRstepPin, HIGH);
    if (debugg == 0) {
      delayMicroseconds(curDelay - disDelayCur);
    }

  }



}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS L
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsL(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int TRstep, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int TRdir, float curDelay) {

  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (TRstep > HighStep)
  {
    HighStep = TRstep;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int TRactive = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (TRstep >= 1)
  {
    TRactive = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int TR_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int TR_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int TR_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int TR_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int TR_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int TRcur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int TR_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int TR_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int TR_SE_2cur = 0;

  int highStepCur = 0;

  float speedSP;
  float moveDist;

  //SET DIRECTIONS

  /// J1 ///
  if (J1dir) {
    digitalWrite(J1dirPin, HIGH);
  }
  else {
    digitalWrite(J1dirPin, LOW);
  }
  /// J2 ///
  if (J2dir) {
    digitalWrite(J2dirPin, LOW);
  }
  else {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3dir) {
    digitalWrite(J3dirPin, LOW);
  }
  else {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4dir) {
    digitalWrite(J4dirPin, HIGH);
  }
  else {
    digitalWrite(J4dirPin, LOW);
  }
  /// J5 ///
  if (J5dir) {
    digitalWrite(J5dirPin, LOW);
  }
  else {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6dir) {
    digitalWrite(J6dirPin, LOW);
  }
  else {
    digitalWrite(J6dirPin, HIGH);
  }

  /// TR ///
  if (TRdir) {
    digitalWrite(TRdirPin, HIGH);
  }
  else {
    digitalWrite(TRdirPin, LOW);
  }


  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;

  ///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
  {

    float distDelay = 60;
    float disDelayCur = 0;


    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J1dir == 0) {
              J1StepM == --J1StepM;
            }
            else {
              J1StepM == ++J1StepM;
            }
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }


    /////// J2 ////////////////////////////////

    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J2dir == 0) {
              J2StepM == --J2StepM;
            }
            else {
              J2StepM == ++J2StepM;
            }
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J3dir == 0) {
              J3StepM == --J3StepM;
            }
            else {
              J3StepM == ++J3StepM;
            }
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }


    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J4dir == 0) {
              J4StepM == --J4StepM;
            }
            else {
              J4StepM == ++J4StepM;
            }
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }


    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J5dir == 0) {
              J5StepM == --J5StepM;
            }
            else {
              J5StepM == ++J5StepM;
            }
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }


    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J6dir == 0) {
              J6StepM == --J6StepM;
            }
            else {
              J6StepM == ++J6StepM;
            }
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }


    /////// TR ////////////////////////////////
    ///find pulse every
    if (TRcur < TRstep)
    {
      TR_PE = (HighStep / TRstep);
      ///find left over 1
      TR_LO_1 = (HighStep - (TRstep * TR_PE));
      ///find skip 1
      if (TR_LO_1 > 0)
      {
        TR_SE_1 = (HighStep / TR_LO_1);
      }
      else
      {
        TR_SE_1 = 0;
      }
      ///find left over 2
      if (TR_SE_1 > 0)
      {
        TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
      }
      else
      {
        TR_LO_2 = 0;
      }
      ///find skip 2
      if (TR_LO_2 > 0)
      {
        TR_SE_2 = (HighStep / TR_LO_2);
      }
      else
      {
        TR_SE_2 = 0;
      }
      /////////  TR  ///////////////
      if (TR_SE_2 == 0)
      {
        TR_SE_2cur = (TR_SE_2 + 1);
      }
      if (TR_SE_2cur != TR_SE_2)
      {
        TR_SE_2cur = ++TR_SE_2cur;
        if (TR_SE_1 == 0)
        {
          TR_SE_1cur = (TR_SE_1 + 1);
        }
        if (TR_SE_1cur != TR_SE_1)
        {
          TR_SE_1cur = ++TR_SE_1cur;
          TR_PEcur = ++TR_PEcur;
          if (TR_PEcur == TR_PE)
          {
            TRcur = ++TRcur;
            TR_PEcur = 0;
            digitalWrite(TRstepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (TRdir == 0) {
              TRStepM == --TRStepM;
            }
            else {
              TRStepM == ++TRStepM;
            }
          }
        }
        else
        {
          TR_SE_1cur = 0;
        }
      }
      else
      {
        TR_SE_2cur = 0;
      }
    }

    // inc cur step
    highStepCur = ++highStepCur;
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    digitalWrite(J5stepPin, HIGH);
    digitalWrite(J6stepPin, HIGH);
    digitalWrite(TRstepPin, HIGH);
    delayMicroseconds(curDelay - disDelayCur);

  }
}

