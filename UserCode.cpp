#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force a single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]
const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;
float const timeConstant_rollRate = 0.04f; // [ s ]
float const timeConstant_pitchRate = timeConstant_rollRate;
float const timeConstant_yawRate = 0.1f; //[s]
float const timeConstant_rollAngle = 0.12f; // [ s ]
float const timeConstant_pitchAngle = timeConstant_rollAngle;
float const timeConstant_yawAngle = 0.2f; //[s]
float const timeConst_horizVel = 2.0f;//[s]

const float rho = 0.01f; // rho for integration
const float dt = 1.0f / 500.0f;  //[s] period between successive calls to MainLoop
Vec3f estGyroBias = Vec3f (0,0,0);//define and initialize estimated GyroBias
Vec3f rateGyro_corr = Vec3f (0,0,0);//define and initialize rateGyro correction
float estRoll = 0;//define and initialize estimated Roll
float estPitch = 0;//define and initialize estimated Pitch
float estYaw = 0;//define and initialize estimated Yaw

//define the respective force fro each motor = [cp1 cp2 cp3 cp4]
float cpi[4] = {0.0f, 0.0f, 0.0f, 0.0f};

//define the elments in  [cp_sigma n1 n2 n3]
float cn[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float desTorque[3] = {0.0f, 0.0f, 0.0f};
float k = 0.01;
float l = 33e-3;
float mixer[4][4] = {{0.25f, 0.25f/(l), 0.25f/(-l), 0.25f/(k)},
    {0.25f, 0.25f/(-l), 0.25f/(-l), 0.25f/(-k)},
    {0.25f, 0.25f/(-l), 0.25f/(l), 0.25f/(k)},
    {0.25f, 0.25f/(l), 0.25f/(l), 0.25f/(-k)}};

// state estimation variables initialization
float estHeight = 0;
float estVelocity_1=0;
float estVelocity_2=0;
float estVelocity_3=0;
float lastHeightMeas_meas=0;
float lastHeightMeas_time=0;

MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Your code goes here!
  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.userInput.buttonBlue" is true if the
  // blue button is pushed, false otherwise.
  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;
  //this if below means evenly adding the 500Hz data in the 1st second.
  bool buttonBlueValue = in.userInput.buttonBlue;
  float desiredSpeed;
  int pwmCommandVal;

  float totalForce;

  //desiredSpeed = speedFromForce(0.07);
  //pwmCommandVal = pwmCommandFromSpeed(desiredSpeed);


  if(in.currentTime<1.0f){
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro/500.0f);
  }
  rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;//define rateGyro correction it is 3-dimension vector and we then use the bias we got to get the correction


  // Deliverable 4.
  float phi_m = (in.imuMeasurement.accelerometer.y) / gravity;
  float theta_m = (-in.imuMeasurement.accelerometer.x) / gravity;

  // Estimate measurements
  estPitch = (1 - rho) * (estPitch + dt*rateGyro_corr.y) + (theta_m * rho);
  estRoll = (1 - rho) * (estRoll + dt*rateGyro_corr.x) + (phi_m * rho);
  estYaw = estYaw + dt*rateGyro_corr.z;

  //height estimator
  //predicton step:
  estHeight = estHeight + dt*estVelocity_3;
  estVelocity_3 = estVelocity_3 + 0*dt; //assume constant

  //correction step:
  float const mixHeight = 0.3f;
  if (in.heightSensor.updated){
    if (in.heightSensor.value<5.0f){
      float hMeas = in.heightSensor.value*cosf(estRoll)*cosf(estPitch);
      estHeight = (1-mixHeight)*estHeight + mixHeight*hMeas;

      float v3Meas = (hMeas - lastHeightMeas_meas) / (in.currentTime - lastHeightMeas_time);
      estVelocity_3 = (1-mixHeight)*estVelocity_3+mixHeight*v3Meas;
      //store tihis measurement for the next velocity uodate
      lastHeightMeas_meas = hMeas;
      lastHeightMeas_time = in.currentTime;
    }
  }

  //horizontal estimator
  //predicton step:
  estVelocity_1 = estVelocity_1 + 0*dt; //assume constant
  estVelocity_2 = estVelocity_2 + 0*dt; //assume constant

  //correction
  float const mixHorizVel = 0.1f;
    if (in.opticalFlowSensor.updated){
      float sigma_1 = in.opticalFlowSensor.value_x;
      float sigma_2 = in.opticalFlowSensor.value_y;

      float div =  (cosf(estRoll)*cosf(estPitch));
      if (div > 0.5f){
        float deltaPredict = estHeight/div;//delta in the equation

        float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
        float v2Meas = (-sigma_2 - in.imuMeasurement.rateGyro.x) * deltaPredict;

        estVelocity_1 = (1-mixHorizVel)*estVelocity_1 + mixHorizVel*v1Meas;
        estVelocity_2 = (1-mixHorizVel)*estVelocity_2 + mixHorizVel*v2Meas;
      }
    }

  //horizontal control
  float desAcc1 = -(1 / timeConst_horizVel) * estVelocity_1;
  float desAcc2 = -(1 / timeConst_horizVel) * estVelocity_2;
  Vec3f desiredAng = Vec3f (0,0,0);//define desired angle as 0 0 0 for initial
  desiredAng.x = -desAcc2/gravity;
  desiredAng.y = desAcc1/gravity;
  desiredAng.z = 0;

  // Vertical
  const float desHeight = 0.5f;
  const float desAcc3 = -2 * dampingRatio_height * natFreq_height * estVelocity_3 - natFreq_height * natFreq_height * (estHeight - desHeight);
  float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));




  Vec3f cmdAngAcc;//define command angular acceleration
  Vec3f cmdAngVel;//define command angular velocity

  //if(buttonBlueValue){
  //  desiredAng = Vec3f(0,0.5236,0);//user push the blue button and the desired angle changes
  //}
  //else{
  //  desiredAng = Vec3f(0,0,0);
  //}
  cmdAngVel.x = -1/timeConstant_rollAngle*(estRoll-desiredAng.x);
  cmdAngVel.y = -1/timeConstant_pitchAngle*(estPitch-desiredAng.y);
  cmdAngVel.z = -1/timeConstant_yawAngle*(estYaw-desiredAng.z);
  cmdAngAcc.x = -1/timeConstant_rollRate*(rateGyro_corr.x-cmdAngVel.x);
  cmdAngAcc.y = -1/timeConstant_pitchRate*(rateGyro_corr.y-cmdAngVel.y);
  cmdAngAcc.z = -1/timeConstant_yawRate*(rateGyro_corr.z-cmdAngVel.z);



//  float totalThrust = 8;//input total thrust
    cn[1] = inertia_xx*cmdAngAcc.x;//calculation angAcc*JBB to obtain torque n1 n2 n3
    cn[2] = inertia_yy*cmdAngAcc.y;//calculation angAcc*JBB to obtain torque n1 n2 n3
    cn[3] = inertia_zz*cmdAngAcc.z;//calculation angAcc*JBB to obtain torque n1 n2 n3
    totalForce = desNormalizedAcceleration*mass;//calculate totalForce c_sigma = m*a
    cn[0] = totalForce;//cn is [c_sigma;n1;n2;n3]
 //   for(int i = 0; i < 4; i++){
  //    for(int j = 0; j < 4; j++){
  //      cpi[i] += mixer[i][j] * cn[j];//derive cpi - motor respective force for motor i
  //    }
  //  }
    cpi[0] = mixer[0][0] * cn[0]+mixer[0][1] * cn[1]+mixer[0][2] * cn[2]+mixer[0][3] * cn[3];
    cpi[1] = mixer[1][0] * cn[0]+mixer[1][1] * cn[1]+mixer[1][2] * cn[2]+mixer[1][3] * cn[3];
    cpi[2] = mixer[2][0] * cn[0]+mixer[2][1] * cn[1]+mixer[2][2] * cn[2]+mixer[2][3] * cn[3];
    cpi[3] = mixer[3][0] * cn[0]+mixer[3][1] * cn[1]+mixer[3][2] * cn[2]+mixer[3][3] * cn[3];
//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y
  outVals.motorCommand1 = 0;//set the initial state as stationary for motor1
  outVals.motorCommand2 = 0;//set the initial state as stationary for motor2
  outVals.motorCommand3 = 0;//set the initial state as stationary for motor3
  outVals.motorCommand4 = 0;//set the initial state as stationary for motor4

  float desiredspeed[4];//define desiredspeed for motor 1 2 3 4
//motor1_respectiveforce = 10;      //debug purpose
//motor2_respectiveforce = 10;
//motor3_respectiveforce = 10;
 //motor4_respectiveforce = 10;
  for (int i=0;i<4;i++){
    desiredspeed[i] = speedFromForce(cpi[i]);//compute the desired speed from desired force motor i
  }

  int pwmcommand[4] = {0,0,0,0};//define desired pmwcommand for motor 1 2 3 4
  for (int i =0;i<4;i++){
    pwmcommand[i] = pwmCommandFromSpeed(desiredspeed[i]);//compute the desired speed from pmwcommand motor i
  }

  if(in.userInput.buttonBlue){//user push blue button and input the pwmcommand to move the UAV
    outVals.motorCommand1 = pwmcommand[0];//motor i moved by pwmcommand[i]
     outVals.motorCommand2 = pwmcommand[1];//
     outVals.motorCommand3 = pwmcommand[2];//
     outVals.motorCommand4 = pwmcommand[3];//
  }
  outVals.telemetryOutputs_plusMinus100 [0] = estRoll ;//1st row of output value in telemetry outputs is estimated roll
  outVals.telemetryOutputs_plusMinus100 [1] = estPitch ;//2nd row of output value in telemetry outputs is estimated roll
  outVals.telemetryOutputs_plusMinus100 [2] = estYaw ;//3rd row of output value in telemetry outputs is estimated roll
  outVals.telemetryOutputs_plusMinus100 [3] = estVelocity_1;//record of command angular acceleration
  outVals.telemetryOutputs_plusMinus100 [4] = estVelocity_2;
  outVals.telemetryOutputs_plusMinus100 [5] = estVelocity_3;
  outVals.telemetryOutputs_plusMinus100 [6] = estHeight;//record of command angular velocity
  outVals.telemetryOutputs_plusMinus100 [7] = desiredAng.x;
  outVals.telemetryOutputs_plusMinus100 [8] = desiredAng.y;
  outVals.telemetryOutputs_plusMinus100 [9] = desNormalizedAcceleration;
  outVals.telemetryOutputs_plusMinus100 [10] = desAcc3;
  // send our attitude estimate back
  //outVals.telemetryOutputs_plusMinus100 [0] = estAtt_roll ;
  //outVals.telemetryOutputs_plusMinus100 [1] = estAtt_ pitch ;
  //outVals.telemetryOutputs_plusMinus100 [2] = estAtt_yaw ;

  //outVals.telemetryOutputs_plusMinus100 [3] = cmdAngAcc.x;//record of command angular acceleration
  //outVals.telemetryOutputs_plusMinus100 [4] = cmdAngAcc.y ;
  //outVals.telemetryOutputs_plusMinus100 [5] = cmdAngAcc.z;
  //outVals.telemetryOutputs_plusMinus100 [6] = cmdAngVel.x;//record of command angular velocity
  //outVals.telemetryOutputs_plusMinus100 [7] = cmdAngVel.y ;
  //outVals.telemetryOutputs_plusMinus100 [8] = cmdAngVel.z;
  //outVals.telemetryOutputs_plusMinus100 [9] = desiredAng.y;//we only changed the pitch angle at first
  //copy the inputs and outputs:


  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;

  return outVals;
}



void PrintStatus() {
//For a quick reference on the printf function, see:
http://www.cplusplus.com/reference/cstdio/printf/
// Note that \n is a "new line" character.
// Also, note that to print a `float` variable, you have to explicitly cast it to
// `double` in the printf function, and explicitly specify precision using something
// like %6.3f (six significant digits, three after the period). Example:
// printf(" exampleVariable_float = %6.3f\n", double(exampleVariable_float));
printf("Time: %6.3f\n", lastMainLoopInputs.currentTime);
//Accelerometer measurement
printf("Acc:");
printf("x=%6.3f, ",
double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
printf("y=%6.3f, ",
double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
printf("z=%6.3f, ",
double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
printf("\n");
//Rate gyro measurement
printf("Gyro:");
printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
printf("\n");
// corrected Gyro measurement
printf("corrected x: %6.3f ", double(rateGyro_corr.x));
printf("y: %6.3f ", double(rateGyro_corr.y));
printf("z: %6.3f ", double(rateGyro_corr.z));
printf("\n");
printf("estRoll: %6.3f ", estRoll);
printf("estPitch: %6.3f ", estPitch);
printf("estYaw: %6.3f \n", estYaw);
//just an example of how we would inspect the last main loop inputs and outputs:
printf("Last main loop inputs:\n");
printf(" batt voltage = %6.3f\n",
double(lastMainLoopInputs.batteryVoltage.value));
printf(" JS buttons: ");
if (lastMainLoopInputs.userInput.buttonRed)
printf("buttonRed ");
if (lastMainLoopInputs.userInput.buttonGreen)
printf("buttonGreen ");
if (lastMainLoopInputs.userInput.buttonBlue)
printf("buttonBlue ");
if (lastMainLoopInputs.userInput.buttonYellow)
printf("buttonYellow ");
if (lastMainLoopInputs.userInput.buttonArm)
printf("buttonArm ");
printf("\n");
printf("Last main loop outputs:\n");
printf(" motor commands: = %6.3f\t%6.3f\t%6.3f\t%6.3f\t\n",
double(lastMainLoopOutputs.motorCommand1),
double(lastMainLoopOutputs.motorCommand2),
double(lastMainLoopOutputs.motorCommand3),
double(lastMainLoopOutputs.motorCommand4));

//printf("Last main loop outputs:\n");
//printf(" desired ang acc = %6.3f\t%6.3f\t%6.3f\t%6.3f\t\n",
//double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[3]),
//double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[4]),
//double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[5]));

//printf(" desired ang vel: = %6.3f\t%6.3f\t%6.3f\t%6.3f\t\n",
//double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[6]),
//double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[7]),
//double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[8]));

//printf(" desired pitch ang: = %6.3f\t\n",
//double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[9]));

printf("Last range = %6.3fm, ", \
  double(lastMainLoopInputs.heightSensor.value));
printf("Last flow: x =  %6.3f , y = %6.3f \n", \
  double(lastMainLoopInputs.opticalFlowSensor.value_x), \
  double(lastMainLoopInputs.opticalFlowSensor.value_y));
//printf("desAcc3: %6.3f",
//  double(outVals.telemetryOutputs_plusMinus100[10]));
}


