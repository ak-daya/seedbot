#include <Servo.h>
#include <Wire.h>

int posX = 0;
int posY = 0;
int posZ = 0;

const byte forceClaw = 0;
const byte forceMoisture;
const byte clawPin = 11;
const byte seedPin = 22;
const byte diggerPin = 10;
const byte moisturePin = 24;
const byte leftEncA = 8;
const byte leftEncB = 7;
const byte rightEncA = 6;
const byte rightEncB = 5;
const byte valvePin = 12;
//const byte infraPin = 0;



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

    void Sweep(int fromAngle, int toAngle)
    {
      if(fromAngle < toAngle){
        for(i = fromAngle; i < toAngle; i++){ 
          servo.write(i);
        }
      }
      else if(fromAngle > toAngle){
        for(i = fromAngle; i > toAngle; i--){
          servo.write(i);
        }
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
      digitalWrite(valvePin, HIGH);
    }
  
    void closeValve(){
      //stop signal;
      digitalWrite(valvePin, LOW);
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
  
    void moveForward(//x,y pos){
      // desired rpm
      int r = 500;
      
      // prop controller Kp
      float Kp = 0.5;
      
      //init global vars
      //rpm reading from the encoder
      int y = 0;
      
      //error
      int e = 0;
      
      //output pwm to motor
      float u_pwm = 0;
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

    void go2next(){
      //go from row n to n+1
    }
};

class Extender      //call sweep fn within
{
  Servo servo;
  int angle = 0;

  public:
    Extender(int servoPin){
		  servo.attach(servoPin);
    }
    
    void Detach(){
      servo.detach();
    }
    
    void actuate(int angle){
      int threshold1, threshold2, threshold3;
      while(analogRead(forcePin) < threshold1){
        servo.write(angle);
        break;
      }
      //write angle to servo;
    }
  
    void freeze(){
      //stop servo
    }
};
