#include <Adafruit_MotorShield.h>
#include <Vex.h>
////Declare Vex object named Robot
Vex Robot;


#define PIN_X 0
#define PIN_Y 1
#define PIN_Z 2


//Create motor objects
Adafruit_DCMotor *motorA = Robot.setMotor(1);
Adafruit_DCMotor *motorB = Robot.setMotor(4);
Adafruit_DCMotor *motorC = Robot.setMotor(3);


void setup() {
//define mode for PINs
  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);
  pinMode(PIN_Z, INPUT);
//set up the serial monitor
  Serial.begin(9600);
  Robot.begin();
  delay(5000);
}


void loop() {
//read from the PINs from joystick
  int x,y,z;
  x=analogRead(PIN_X);
  y=analogRead(PIN_Y);
  z=analogRead(PIN_Z);
//give order to motors based on the reading of PINs
  if(y==0){
    Robot.moveTank(motorA, motorB, 50, 50, 2);
  }else if(y==1023){
    Robot.moveTank(motorA, motorB, -50, -50, 2);
  }else if(x==0){
    Robot.moveMotor(motorB, 50, 1);
  }else if(x==1023){
    Robot.moveMotor(motorA, 50, 1);
  }
  Serial.print("X=");
  Serial.print(x);
  Serial.print("\tY=");
  Serial.print(y);
  Serial.print("\tZ=");
  Serial.println(z);
  delay(1000);
}
