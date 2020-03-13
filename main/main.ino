#include <Servo.h>
#include <Wire.h>

/*positional variables*/
int posX = 0;
int posY = 0;
int posZ = 0;

/*soil force threshold qualities*/
const int soil = 200;
const int toughSoil = 400;

/*pin assignment*/
const byte forceClaw = 0;
const byte forceMoist;
/*-----------------------------*/
const byte clawServoPin = 11;
const byte seedServoPin = 22;
const byte diggerServoPin = 10;
const byte moistServoPin;
/*-----------------------------*/
const byte leftEncA = 8;
const byte leftEncB = 7;
const byte rightEncA = 6;
const byte rightEncB = 5;
/*-----------------------------*/
const byte valvePin = 12;
/*-----------------------------*/
const byte infraPin = 0;


/*=====CLASS DEFINITIONS=====*/
class Sweeper     //for claw and for seed dispenser
{
  private:
    Servo servo;
    byte lastAngle;
  public:
    Sweeper(byte servoPin) {
      servo.attach(servoPin);
      reset();
    }

    void reset(){
      servo.write(90);
      lastAngle = 90;
    }
    
    void Sweep(int toAngle){
      if(lastAngle <= toAngle){
        for(int i = lastAngle; i < toAngle; i++)  {servo.write(i);}
        lastAngle = toAngle;
      }
      else if(lastAngle > toAngle){
        for(int i = lastAngle; i > toAngle; i--)  {servo.write(i);}
        lastAngle = toAngle;
      }
    }
};

class Extender : public Sweeper      //reacher class inherits public fns of Sweeper
{
  private:
    byte forcePin;
    byte servoPin;
    byte lastAngle;
  public:
    Extender(byte servoPin, byte forcePin):Sweeper(servoPin){
      forcePin = forcePin;
    }
  
    void Actuate(int lastAngle, int toAngle, int threshold){
      while(analogRead(forcePin) < threshold){
			  servo.write(lastAngle);
        lastAngle++;
			}
    }

    void Return(int fromAngle, int toAngle) {Sweep(fromAngle, toAngle);}
};

class MoistureSensor : public Extender
{
  private:
    byte moistPin;
    byte reading;
    unsigned long lastTime;
  public:
    MoistureSensor(byte servoPin, byte forcePin, byte moistPin):Extender(servoPin, forcePin){
      moistPin = moistPin;
    }

    int Update(){
      reading = analogRead(moistPin);
      return reading;
    }
};

class Valve
{
  int valvePin;

  public:
    Valve(byte pin){
      valvePin = pin;
    }
    
    void openValve()  {digitalWrite(valvePin, HIGH);} //actuation signal
  
    void closeValve() {digitalWrite(valvePin, LOW);}  //stop signal;
};
//TO BE COMPLETED
class Wheel
{
  private:
    byte encoderAPin;
    byte encoderBPin;
    byte pwmSpeed;

  public:
    Wheel(byte encoderA, byte encoderB){
      encoderAPin = encoderA;
      encoderBPin = encoderB;
      pinMode(encoderAPin, INPUT);
      pinMode(encoderBPin, INPUT);
      //need to find a way to connect I2C in this class (pins: SDA - 20, SCL - 21)
    }
    
    int getSpeed(){
      int rpm;
      //access encoder, run calculations
      /*y = analogRead(forceClaw);    //store measured force
        e = r - y;    //e = r - y
        u_angle = Kp*e;
        
        u_angle = bound(u_angle, -180, 180);
        //myServo.write(u_angle);
        
        Serial.print(y);
        Serial.print('\t');
        Serial.print(r);
        Serial.print('\t');
        Serial.println(e);
        
        delay(50);*/
      return rpm;
    }
  
    void moveForward(/*x,y pos*/){
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

/*create objects*/
Sweeper seeder(seedServoPin), claw(clawServoPin);
Extender digger(diggerPin, forceClaw);
MoistureSensor moistSensor(moistServo, forceMoist, moisturePin);
Wheel left(leftEncA, leftEncB), right(rightEncA, rightEncB);
Valve waterOut(valvePin);

/*=====SETUP=====*/
void setup() {
	
	Serial.begin(9600);
	//myServo.attach(10);
	//myServo.write(0);
 
	/*~~~INIT~~~*/
	//pinMode(infraPin, INPUT);
}

/*=====MAIN=====*/
void loop() {
	
}

float bound(float x, float x_min, float x_max){
	if(x < x_min){x = x_min;}
	if(x > x_max) {x = x_max;}
	return x;
}
