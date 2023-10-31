#include "UtilityFunctions.hpp"
#include <cstdio>
int pwmCommandFromSpeed(float desiredSpeed_rad_per_sec) {
  // Replace these two coefficients with what you get
  // in the experiment. Note the trailing "f" after the
  // number -- this ensures that we use single precision
  // floating point (rather than double precision, which
  // would be substantially slower on the microcontroller).

  float a = -69.3328f;  // the zeroth order term
  float b = 0.1446f;  // the first order term

  return int(a + b * desiredSpeed_rad_per_sec);
}

float speedFromForce(float desiredForce_N) {
  // replace this with your determined constant:
  // Remember to add the trailing "f" for single
  // precision!
  float const propConstant = 3.845e-8f;

  //we implement a safety check,
  //  (no sqrtf for negative numbers)
  if (desiredForce_N <= 0) {
    return 0.0f;
  }

  return sqrtf(desiredForce_N / propConstant);
}



float TotalForcetoSingleForce(float totalForce,float n1,float n2, float n3){
  //the function receiving the total force and obtain the respective force to each motors
  //Output[cp1'cp2;cp3;cp4] = {MIXMAT}*{c_sigma;n1;n2;n3}
  //float MixMat[4][4]={0.25,1/(4*33*10^(-3)),-1/(4*33*10^(-3)),25,0.25,-1/(4*33*10^(-3)),-1/(4*33*10^(-3)),-25,0.25,-1/(4*33*10^(-3)),1/(4*33*10^(-3)),25,0.25,1/(4*33*10^(-3)),1/(4*33*10^(-3)),-25}; change 10^-3 -> e-3
  float MixMat[4][4] = {0.25,0.25/(33e-3),-0.25/(33e-3),25,0.25,-0.25/(33e-3),-0.25/(33e-3),-25,0.25,-0.25/(33e-3),0.25/(33e-3),25,0.25,0.25/(33e-3),0.25/(33e-3),-25};
  //do the M*[c_sigma,ni...] calculation
  float input[4]={totalForce,n1,n2,n3};//input [c_sigma n1 n2 n3]
  //printf("function called constants %6.3f \t %6.3f \t %6.3f \t %6.3f \t",totalForce,n1,n2,n3)
  float output[4] = {{0.0f}};

  for (int i=0;i<4;i++){//calculate matrix multiplication
      for (int k=0;k<4;k++){
      output[i] = MixMat[i][k]*input[k]+output[i];//a = sigma(i=1->i=4)sigma(j=1->j=4)(m_ij*input_j1)
      }

  }
  //printf("function outputs %6.3f \t %6.3f \t %6.3f \t %6.3f \t",output[0],output[1],output[2],output[3])
  //test whether the matrix mult is err
  //output[1] = totalForce/4;
  //output[2] = totalForce/4;
  //output[3] = totalForce/4;
  //output[4] = totalForce/4;
  return (output[0],output[1],output[2],output[3]);//output cp1 cp2 cp3 cp4 desired motor forces respectively
}

