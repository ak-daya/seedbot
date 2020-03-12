#include <Servo.h>
#include <Wire.h>


const int forceClaw = 0;
const int forceMoisture;
const int clawPin = 11;
const int seedPin = 22;
const int diggerPin = 10;
const int moisturePin = 24;
const int leftEncA = 8;
const int leftEncB = 7;
const int rightEncA = 6;
const int rightEncB = 5;
const int valvePin = 12;
//const int infraPin = 0;

// desired force
int r = 500;

// prop controller Kp
float Kp = 0.5;

//init global vars
//reading from the forcesensor
int y = 0;

//error
int e = 0;

//output angle to servo
float u_angle = 0;

Servo myServo;
  /*~~~CREATE OBJS~~~*/
  //Sweeper seeder(seedPin), claw(clawPin);
  //Extender digger(diggerPin, forceClaw), moistureServo(moisturePin, forceMoisture);
  //Wheel left(leftEncA, leftEncB), right(rightEncA, rightEncB);
  //Valve waterOut(valvePin);
/*=====SETUP=====*/
void setup() {
	
	Serial.begin(9600);
	myServo.attach(10);
  myServo.write(0);

	/*~~~INIT~~~*/
	//pinMode(infraPin, INPUT);
	//moistureServo.Attach(9);      //moisture sensor needs to be a class extending Extender class to allow moisture sensing as a method
}

/*=====MAIN=====*/
void loop() {
	y = analogRead(forceClaw);		//store measured force
	e = r - y;		//e = r - y
	u_angle = Kp*e;
	
	u_angle = bound(u_angle, -180, 180);
	myServo.write(u_angle);
	
	Serial.print(y);
	Serial.print('\t');
	Serial.print(r);
	Serial.print('\t');
	Serial.println(e);
	
	delay(50);
}

float bound(float x, float x_min, float x_max){
	if(x < x_min){x = x_min;}
	if(x > x_max) {x = x_max;}
	return x;
}

/*=====CLASS DEFINITIONS=====*/
/*
class Sweeper     //for claw and for seed dispenser
{
  Servo servo;
  int angle = 0;
  
  public:
    Sweeper(int servoPin){
		servo.attach(servoPin);
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
      //12*298 = 3576
    }
  
    void brake(){
      //write brake sequence to motor
    }

    void turnRight(){
      //
    }

    void turnLeft(){
      //
    }
};

class Extender
{
  Servo servo;
  int angle = 0;

  public:
    Extender(int servoPin, int forcePin){
		pinMode(forcePin, INPUT);
		servo.attach(servoPin);
    }
    
    void Detach(){
      servo.detach();
    }
    
	int checkSensor(int forcePin){
      int force = analogRead(forcePin);
      return force;
    }    
    void actuate(int angle){
      while(checkSensor() < 500){
        servo.write(angle);
      }
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
*/
