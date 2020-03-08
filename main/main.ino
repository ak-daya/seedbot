#include <Servo.h>
#include <Wire.h>

/*=====CLASS DEFINITIONS=====*/
class Sweeper
{
  Servo servo;
  int angle = 0;
  
  public:
    Sweeper(){}

    void Attach(int pin)
    {
      servo.attach(pin);
    }

    void Detach()
    {
      servo.detach();
    }

    void SweepOut()
    {
      for(angle = 0; angle < 65; angle++){ 
        servo.write(angle);
      }
    }

    void SweepIn()
    {
      for(angle = 65; angle > 0; angle--){
        servo.write(angle);
      }
    }
};

class Wheel
{
  int encoderAPin;
  int encoderBPin;
  int pwmSpeed;

  public:
    Wheel(int encoderA, int encoderB){
      encoderAPin = encoderA;
      encoderBPin = encoderB;
      pinMode(encoderAPin, INPUT);
      pinMode(encoderBPin, INPUT);
      //need to find a way to connect I2C in this class (pins: SDA - 20, SCL - 21)
    }
    
    int getSpeed(){
      int rpm;
      //access encoder, run calculations
      return rpm;
    }
  
    void moveForward(int pwmSpeed){
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
  int angle = 0;

  public:
    Extender(int forcePin){
      fsrPin = forcePin;
      pinMode(fsrPin, INPUT);
    }
    
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
    
    void actuate(int angle){
      //write angle to servo;
    }
  
    void freeze(){
      //stop servo
    }
};

class Valve
{
  int valvePin;

  public:
    Valve(int pin){
      valvePin = pin;
    }
    
    void openValve(){
      //actuation signal;
    }
  
    void closeValve(){
      //stop signal;
    }
};


/*=====VAR INIT=====*/




/*=====SETUP=====*/
void setup() {
  /*~~~CREATE OBJS~~~*/
  Sweeper seedServo, claw;
  Extender digger(23), moistureServo(24);
  Wheel left(8, 7), right(6, 5);
  Valve waterOut(12);
  /*~~~SET PINS~~~*/
  int infraPin = 0;
  /*~~~INIT~~~*/
  pinMode(infraPin, INPUT);
  seedServo.Attach(22);
  claw.Attach(11);
  digger.Attach(10);
  moistureServo.Attach(9);      //moisture sensor needs to be a class extending Extender class to allow moisture sensing as a method
  
  Serial.begin(9600);
}

/*=====MAIN=====*/
void loop() {

}
