#include <Servo.h>
#include <Wire.h>


/*=====CLASS DEFINITIONS=====*/
class Sweeper
{
  Servo servo;
  int angle;
  const int updateInterval = 15;
  int lastUpdate;
  
  public:
    Sweeper(int interval){}

    void Attach(int pin)
    {
      servo.attach(pin);
    }

    void Detach()
    {
      servo.detach();
    }

    void Sweep()
    {
      if((millis() - lastUpdate) > updateInterval)
      {
        lastUpdate = millis();
        for(angle = 0; angle < 90; angle++){ 
          servo.write(angle);
        }
        
      }
    }
};

class Wheel
{
  int encoderA;
  int encoderB;
  int motorPin;
  int pwmSpeed;

  int getSpeed(){
    int rpm;
    //access encoder, run calculations
    return rpm;
  }

  void moveForward(pwmSpeed){
    //write pwm speed to motor
  }

  void brake(){
    //write brake sequence to motor
  }
};

class Extender
{
  Servo servo;
  int fsrPin;
  int fsrData;
  int angle;

  void Attach(int pin)
    {
      servo.attach(pin);
    }

  void Detach()
  {
    servo.detach();
  }
   
  int checkSensor(){
    int force;
    //get force reading from sensor
    return force;
  }
  
  void actuate(angle){
    //write angle to servo;
  }

  void freeze(){
    //stop servo
  }
};

class Valve
{
  int valvePin;

  void openValve(){
    //actuation signal;
  }

  void closeValve(){
    //stop signal;
  }
};


/*=====VAR INIT=====*/
Rotator servo1(15);
int infraPin = 0;

Servo myServo;


/*=====SETUP=====*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myServo.attach(11);
  pinMode(infraPin, INPUT);
}



/*=====MAIN=====*/
void loop() {
  // put your main code here, to run repeatedly:
  //output = analogRead(infraPin);
  //Serial.println(output);
  myServo.write(90);
  myServo.write(89);
}
