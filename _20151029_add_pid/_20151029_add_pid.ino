#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include<PID_v1.h>


//#define ONE_WIRE_BUS 9

//Definition for MAX31855
#define DO 5
#define CS 6
#define CLK 7
Adafruit_MAX31855 thermocouple(CLK, CS, DO);
//definition for using hardware SPI
//#define CS 7
//Adafruit_MAX31855 thermocouple (CS);

//Definition for PID for MAX
double MAX_Kp=2, MAX_Ki=5, MAX_Kd=1;
double setpoint=25;
double MAX_Input, MAX_Output;
PID MAX_PID(&MAX_Input, &MAX_Output, &setpoint, MAX_Kp, MAX_Ki, MAX_Kd, DIRECT);
int WindowSize = 1000; 
unsigned long windowStartTime;
float ctrlkp, ctrlti, ctrltd;

double MAX_Temp;


boolean MAXstate;
String ctrlcmd;
boolean outputstate;
boolean MAXPIDstate;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //sensors.begin();
    //simplified wiring to MAX31855
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(2, HIGH);
  pinMode(10, OUTPUT);
  MAXstate = false;
  //initialization for PID
  //MAX_PID.SetTunings(MAX_Kp,MAX_Ki,MAX_Kd);
  //Serial.println("pid parameters");
  //MAX_PID.SetSampleTime(1000);
  //MAX_PID.SetOutputLimits(0, WindowSize);
  MAX_PID.SetMode(AUTOMATIC);

  
}

void deciphercmd() {
  char sysid=ctrlcmd.charAt(0);
  switch (sysid) {
    case '1':
      MAXstate=true;
      MAXPIDstate=false;
      break;
    case '0':
      MAXstate=false;
      MAXPIDstate=false;
      break;
     case '2':
      MAXstate=false;
      MAXPIDstate=true;
      break;
  }
  int commaIndex = ctrlcmd.indexOf(',');
  int secondcommaIndex = ctrlcmd.indexOf(',', commaIndex+1);
  int thirdcommaIndex = ctrlcmd.indexOf(',', secondcommaIndex+1);
  int fourthcommaIndex = ctrlcmd.indexOf(',', thirdcommaIndex+1);
  //String setpointstring = ctrlcmd.substring(secondcommaIndex+1);
  String setpointstring = ctrlcmd.substring(commaIndex+1, secondcommaIndex);
  setpoint = setpointstring.toFloat();
  String ctrlkpstring = ctrlcmd.substring(secondcommaIndex+1, thirdcommaIndex);
  ctrlkp = ctrlkpstring.toFloat();
  String ctrltistring = ctrlcmd.substring(thirdcommaIndex+1, fourthcommaIndex);
  ctrlti = ctrltistring.toFloat();
  String ctrltdstring = ctrlcmd.substring(fourthcommaIndex+1);
  ctrltd = ctrltdstring.toFloat();

  //decipher the pid parameters
  
}
void getReadings() {
  Serial.flush();
  unsigned long time = millis();
  MAX_Temp = thermocouple.readCelsius();
  //Serial.print("Time,");
  Serial.print(time);
  Serial.print(",");
  Serial.print(outputstate);
  Serial.print(",");
  Serial.println(MAX_Temp);
}
void loop() {
  //Serial.print(setpoint-MAX_Temp);
  if ((MAXstate == true) && ((setpoint-MAX_Temp)>2)){
    //far away from setpoint
    analogWrite(10, 255);
    outputstate = true;
  }
  else if ((MAXstate == true) && ((setpoint-MAX_Temp) >=0) &&  ((setpoint-MAX_Temp) <= 2)){
    int a = (setpoint-MAX_Temp) / 4 * 255;
    analogWrite(10, a);
    outputstate = true;
  }
  else if (MAXPIDstate == true){
    //start PID process
    MAX_Input = MAX_Temp;
    MAX_Kp = ctrlkp;
    MAX_Ki = ctrlti;
    MAX_Kd = ctrltd;
    MAX_PID.SetTunings(MAX_Kp, MAX_Ki, MAX_Kd);
    //MAX_PID.SetMode(AUTOMATIC);
    MAX_PID.Compute();
    analogWrite(10, MAX_Output);
    

  }
  else {
    analogWrite(10, 0);
    outputstate = false;
  }
  getReadings();
  if (Serial.available()>0) {
    ctrlcmd = Serial.readString();
    deciphercmd();
  }
  delay(400);
  }

