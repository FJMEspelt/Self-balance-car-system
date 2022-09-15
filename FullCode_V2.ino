#include <Wire.h>
#include <math.h>
 
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;

//Pins for motor speed control
int motorW1pin1 = 2;
int motorW1pin2 = 3;

int motorW2pin1 = 5;
int motorW2pin2 = 4;

int motorS1pin1 = 6;
int motorS1pin2 = 7;

int motorS2pin1 = 8;
int motorS2pin2 = 9;

int motorS3pin1 = 10;
int motorS3pin2 = 11;

int motorS4pin1 = 12;
int motorS4pin2 = 13;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;
float Setpoint = 0;

float outputWheel1,outputWheel2,outputBed1,outputBed2;
float outputNew1_1, outputNew1_2, outputNew2_1, outputNew2_2;
float outputM1_1, outputM1_2, outputM2_1, outputM2_2;


float KpM1_1 = 1.05;
float KdM1_1 = 0;
float KiM1_1 = 0.01;

float KpM1_2 = 1.05;
float KdM1_2 = 0;
float KiM1_2 = 0.01;

float KpM2_1 = -1.05;
float KdM2_1 = -0;
float KiM2_1 = -0;

float KpM2_2 = 1.05;
float KdM2_2 = 0;
float KiM2_2 = 0.01;

float Kp = -1.05;
float Kd = -0;
float Ki = -0.03;

float Kp1 = 1.05;
float Kd1 = 0;
float Ki1 = 0.03 ;

// variables internas del controlador
unsigned long currentTime, previousTime;
unsigned long currentTime2, previousTime2;
double elapsedTime, elapsedTime2;
double error, lastError, cumError, rateError;
double error1, lastError1, cumError1, rateError1;
double error2, lastError2, cumError2, rateError2;


 
void setup(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);

  pinMode(motorW1pin1, OUTPUT);
  pinMode(motorW1pin2, OUTPUT);
  pinMode(motorW2pin1, OUTPUT);
  pinMode(motorW2pin2, OUTPUT);

  pinMode(motorS1pin1, OUTPUT);
  pinMode(motorS1pin2, OUTPUT);
  pinMode(motorS2pin1, OUTPUT);
  pinMode(motorS2pin2, OUTPUT);

  pinMode(motorS3pin1, OUTPUT);
  pinMode(motorS3pin2, OUTPUT);
  pinMode(motorS4pin1, OUTPUT);
  pinMode(motorS4pin2, OUTPUT);

  delay(4000);
}
void loop(){
  timer = millis();
  delay(10);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
 
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

//  delay((timeStep*1000) - (millis() - timer));                        Delay añadido por modulo, no decidido
  PIDcontrol(z,x,y);
  Serial.println("-----------------------------------------");
  Serial.print("AngleX= ");
  Serial.println(x);
 
  Serial.print("AngleY= ");
  Serial.println(y);
 
  Serial.print("AngleZ= ");
  Serial.println(z);
  

  Serial.print(" PID_Wheel 1 = ");
  Serial.print(outputWheel1);
  Serial.print(" PID_Wheel 2 = ");
  Serial.print(outputWheel2);  
  Serial.print(" PID_Bed 1 = ");
  Serial.print(outputBed1);
  Serial.print(" PID_Bed 2 = ");
  Serial.print(outputBed2);

  //Control of one side of the bed/
  if (outputBed1 >= 0){
    analogWrite(motorS1pin1,outputBed1);
    analogWrite(motorS1pin2,0);
    analogWrite(motorS2pin1,outputBed1);
    analogWrite(motorS2pin2,0);
  } 
  if (outputBed1 < 0){
    outputBed1 = -1*outputBed1;
    analogWrite(motorS1pin1,0);
    analogWrite(motorS1pin2,outputBed1);
    analogWrite(motorS2pin1,0);
    analogWrite(motorS2pin2,outputBed1);
    }
// Control of   the other side of the bed 
  if (outputBed2 >= 0){
    analogWrite(motorS3pin1,outputBed2);
    analogWrite(motorS3pin2,0);
    analogWrite(motorS4pin1,outputBed2);
    analogWrite(motorS4pin2,0);
  } 
  if (outputBed2 < 0){
    outputBed2 = -1*outputBed2;
    analogWrite(motorS3pin1,0);
    analogWrite(motorS3pin2,outputBed2);
    analogWrite(motorS4pin1,0);
    analogWrite(motorS4pin2,outputBed2);
    }


  //Wheel control
  if (outputWheel1 >= 0){
    analogWrite(motorW1pin1,0);
    analogWrite(motorW1pin2,outputWheel1);
  } 
  if (outputWheel1 < 0){
    outputWheel1 = -1*outputWheel1;
    analogWrite(motorW1pin1,outputWheel1);
    analogWrite(motorW1pin2,0);
    }

  if (outputWheel2 >= 0){
    analogWrite(motorW2pin1,0);
    analogWrite(motorW2pin2,outputWheel2);
  } 
  if (outputWheel2 < 0){
    outputWheel2 = -1*outputWheel2;
    analogWrite(motorW2pin1,outputWheel2);
    analogWrite(motorW2pin2,0);
    }
}

float PIDcontrol(float z, float x, float y){
        currentTime = millis();                               // time
        elapsedTime = (double)(currentTime - previousTime);     // elapsedTime
        
//        // z axis control
//        if (z<=200 && z>=290) {
//          error = 240 - z;                               // determine the error between the output and the desired output
//          cumError += error * elapsedTime;                      // integral of the error
//          rateError = (error - lastError) / elapsedTime;         // derivative of the error
//          float outputM1_1 = KpM1_1*error + KiM1_1*cumError + KdM1_1*rateError;     // Calculate output of PID
//          float outputM2_1 = KpM2_1*error + KiM2_1*cumError + KdM2_1*rateError;     // Calculate output of PID
//          lastError = error; 
//          outputNew1_1 = outputM1_1;
//          outputNew2_1 = outputM2_1;
//        } else { 
//          outputNew1_1 = 0;
//          outputNew2_1 = 0;
//        }
//
//        Serial.print(" PARTE 2= ");
//        Serial.print(outputNew1_2);
        
        // x axis control
        if (x<=355 && x>=5) {
          if (x<=40){
              error1 = 0 - x; // determine the error between the output and the desired output
          } else if (x>=300){
            error1 = 360 - x; // determine the error between the output and the desired output
          }
          Serial.print(" error1 ");
          Serial.print(error1);
          cumError1 += error1 * elapsedTime;                      // integral of the error
          rateError1 = (error1 - lastError1) / elapsedTime;         // derivative of the error
          float outputM1_2 = KpM1_2*error1 + KiM1_2*cumError1 + KdM1_2*rateError1;
          float outputM2_2 = KpM2_2*error1 + KiM2_2*cumError1 + KdM2_2*rateError1;
          if (error1<0){
            outputNew1_2 = -outputM1_2;
            outputNew2_2 = -outputM2_2;
            } else {
            outputNew1_2 = outputM1_2;
            outputNew2_2 = outputM2_2;
              }
        } else {
          outputNew1_2 = 0;
          outputNew2_2 = 0;
        }

        outputWheel1 = outputNew1_1 + outputNew1_2;
        outputWheel2 = outputNew2_1 + outputNew2_2;

        Serial.print("  ");
        Serial.print(outputNew1_2);

        if(y>=2 && y <= 358){
          if (y<=80) {
          error2 = 0 - y;                               // determine the error between the output and the desired output
          } else {
          error2 = 360 - y; 
          }
          cumError2 += error2 * elapsedTime;                      // integral of the error
          rateError2 = (error2 - lastError2) / elapsedTime;         // derivative of the error
          outputBed1 = Kp*error2 + Ki*cumError2 + Kd*rateError2;     // Calculate output of PID
          outputBed2 = Kp1*error2 + Ki1*cumError2 + Kd1*rateError2;
          lastError2 = error2;

          // NUEVO PARA SOLUCIONAR ERROR DE CAMBIO DE SENTIDO
          if (y<=80 && outputBed1<0){
            outputBed1 = -1*outputBed1;
            outputBed2 = -1*outputBed2;
            }
          if (y>=300 && outputBed1>0) {
            outputBed1 = -1*outputBed1;
            outputBed2 = -1*outputBed2;
            }
          // TERMINA NUEVO
          
        } else {
          outputBed2 = 0;
          outputBed1 = 0;
          }

        lastError1 = error1;
        previousTime = currentTime;

        if (outputWheel1>=255) {
          outputWheel1 = 255;
        } else if (outputWheel1<=-255) {
          outputWheel1 = -255;
          }
          
        if (outputWheel2>=255) {
          outputWheel2 = 255;
        }  else if (outputWheel2<=-255) {
          outputWheel2 = -255;
          }
   
        if (outputBed1>=255) {
          outputBed1 = 255;
        }  else if (outputBed1<=-255) {
          outputBed1 = -255;
          }

        if (outputBed2>=255) {
          outputBed2 = 255;
        }  else if (outputBed2<=-255) {
          outputBed2 = -255;
          }
        outputWheel1= int(outputWheel1);
        outputWheel2= int(outputWheel2);

        outputBed1= int(outputBed1);
        outputBed2= int(outputBed2);
        
        return outputWheel1,outputWheel2,outputBed1,outputBed2;
          
}
