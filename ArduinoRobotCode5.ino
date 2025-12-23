#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ramp.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Global variable
char c;
String DataIn;
int servo1Degree, servo2Degree, servo3Degree, servo4Degree, servo5Degree, servo6Degree, servoDelay;
int indexOfA, indexOfB, indexOfC, indexOfD, indexOfE, indexOfF, indexOfG;
int pulseWithAxis1, pulseWithAxis2, pulseWithAxis3, pulseWithAxis4, pulseWithAxis5, pulseWithAxis6;


int Axis1 = 1;
int Axis2 = 2;
int Axis3 = 3;
int Axis4 = 4;
int Axis5 = 5;
int Axis6 = 6;

rampInt   RampAxis1;
rampInt   RampAxis2;
rampInt   RampAxis3;
rampInt   RampAxis4;
rampInt   RampAxis5;
rampInt   RampAxis6;

//Button/fan Variable
int ButtonPin = 10;
int FanRight = 11;
int FanLeft = 12;
int NewValue;
int OldValue = 0;
String LedState = "OFF";



void setup() 
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);

//Button/fan setup
  pinMode(ButtonPin, INPUT_PULLUP);
  pinMode(FanRight, OUTPUT);
  pinMode(FanLeft, OUTPUT);

  pwm.begin();
  pwm.setPWMFreq(60); 


}

void loop() 
{
//Button/fan loop
NewValue = digitalRead(ButtonPin);
if(NewValue == 0 && OldValue == 0)
{
  if(LedState == "OFF")
  {
    digitalWrite(FanRight, HIGH);
    digitalWrite(FanLeft, HIGH);
    LedState = "ON";
  }
  else
  {
    digitalWrite(FanRight, LOW);
    digitalWrite(FanLeft, LOW);
    LedState = "OFF";
  }
  delay(500);
  OldValue = NewValue;
}



  Receive_serial_Data();
  if(c=='\n')
  {
      Parse_theData();
      c=0;
      DataIn="";
  }
  
  RampPulseWith();

}


void Receive_serial_Data()
{
  while(Serial.available()>0)
  {
    c=Serial.read();
    if(c=='\n')   {break;}
    else          {DataIn+=c;}

  }
}

void Parse_theData()
{

  String str_servo1Degree, str_servo2Degree, str_servo3Degree, str_servo4Degree, str_servo5Degree, str_servo6Degree, str_servoDelay;

  indexOfA = DataIn.indexOf("A");
  indexOfB = DataIn.indexOf("B");
  indexOfC = DataIn.indexOf("C");
  indexOfD = DataIn.indexOf("D");
  indexOfE = DataIn.indexOf("E");
  indexOfF = DataIn.indexOf("F");
  indexOfG = DataIn.indexOf("G");

  if(indexOfA > -1){ str_servo1Degree = DataIn.substring(0, indexOfA);            servo1Degree = str_servo1Degree.toInt();
  pulseWithAxis1 = map(servo1Degree, 0, 270, 90, 650);} //650

  if(indexOfB > -1){ str_servo2Degree = DataIn.substring(indexOfA+1, indexOfB);   servo2Degree = str_servo2Degree.toInt();
  pulseWithAxis2 = map(servo2Degree, 0, 180, 110, 650);}

  if(indexOfC > -1){ str_servo3Degree = DataIn.substring(indexOfB+1, indexOfC);   servo3Degree = str_servo3Degree.toInt();
  pulseWithAxis3 = map(servo3Degree, 0, 270, 95, 670);}

  if(indexOfD > -1){ str_servo4Degree = DataIn.substring(indexOfC+1, indexOfD);   servo4Degree = str_servo4Degree.toInt();
  pulseWithAxis4 = map(servo4Degree, 0, 180, 200, 550);} //test

  if(indexOfE > -1){ str_servo5Degree = DataIn.substring(indexOfD+1, indexOfE);   servo5Degree = str_servo5Degree.toInt();
  pulseWithAxis5 = map(servo5Degree, 0, 200, 200, 550);}

  if(indexOfF > -1){ str_servo6Degree = DataIn.substring(indexOfE+1, indexOfF);   servo6Degree = str_servo6Degree.toInt();
  pulseWithAxis6 = map(servo6Degree, 0, 90, 200, 460);}

  if(indexOfG > -1){ str_servoDelay = DataIn.substring(indexOfF+1, indexOfG);   servoDelay = str_servoDelay.toInt();}


}

void moveMotor(int pulseWidthOut, int motorOut)
{
  
  pwm.setPWM(motorOut, 0, pulseWidthOut);

}



void RampPulseWith()
{
  RampAxis1.go(pulseWithAxis1, servoDelay, LINEAR, ONCEFORWARD);
  RampAxis2.go(pulseWithAxis2, servoDelay, LINEAR, ONCEFORWARD);
  RampAxis3.go(pulseWithAxis3, servoDelay, LINEAR, ONCEFORWARD);
  RampAxis4.go(pulseWithAxis4, servoDelay, LINEAR, ONCEFORWARD);
  RampAxis5.go(pulseWithAxis5, servoDelay, LINEAR, ONCEFORWARD);
  RampAxis6.go(pulseWithAxis6, servoDelay, LINEAR, ONCEFORWARD);

  while (RampAxis1.isRunning())
  {
    moveMotor(RampAxis1.update(), Axis1);
    moveMotor(RampAxis2.update(), Axis2);
    moveMotor(RampAxis3.update(), Axis3);
    moveMotor(RampAxis4.update(), Axis4);
    moveMotor(RampAxis5.update(), Axis5);
    moveMotor(RampAxis6.update(), Axis6);
  }



}


