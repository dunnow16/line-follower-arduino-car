/*********************************************************************************************************************
 * Program: Line follower for Arduino robot.
 *
 * Written by: Owen Dunn
 *
 * Date: December 12, 2015
 *
 * Input: Sensor IR reflectance readings
 *
 * Output: DC motor input (to turn left/right, go forward/backward)
 *
 * Description: Robot uses a DC motor, our robot uses a shield, polulu QTR reflectence sensors used, the sensor has
 * 6 I/O and provide (digital or analog?) output. The sensors are in an array. This output is interpreted by the code
 * to make a decision on which direction the robot will go. A signal will then be sent to the respective DC motors to
 * go forward, pivot left or right, or backward. The code will continue to read the sensor values as the robot moves.
 *
 * (below is unresolved
 * Should the robot move forward/backward in set incremental distances with each sensor reading to go f/back. OR The
 * robot is capable to continuously reading values and moving forward at the same time?
 *
 * Sensor: makes analog measurements of IR reflectance, 0 to 5000 in 1000 increments to get to sensors 1-6, each
 * sensor provides separate digital(?) I/O
 *
 * Turning: use only the front/back two wheels? Would front or back be better? I think front.
 *
 * Forward/Backward: use all four DC motors
 *
 ********************************************************************************************************************/
#include <Arduino.h> //don't need?
#include <AFMotor.h>
#include <QTRSensors.h> //include the library for the polulu QTR reflectence sensors (array of 6 used), don't need if
                        //don't use qtr functions (can simply take data inputs)
#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

void leftmotorRPM(int RPM);
void rightmotorRPM(int RPM);
void Forward(int Dist, int Speed);
void Backward(int Dist, int Speed);
void PivotLeft(int Angle, int Speed);
void PivotRight(int Angle, int Speed);

QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

AF_DCMotor Motor1(1, MOTOR12_64KHZ); //4 motors are used, need 4 lines like this?

//intialize signals to servos (not used for DC motor)
//Servo leftServo;
//Servo rightServo;

void setup() {
  Serial.begin(9600); //initialize serial input/output
  int sec; //seconds
  int pulse; //a pulse (in ms) sent to the DC motor
  int Speed; //speed for the DC motor



  //initialize the pins for the four DC motors



  //leftServo.attach(10);
  //rightServo.attach(11);

  //insert an algorithm to follow the lines
  //input the RPM command to run at 20 rpm
  //Forward(10,40);
  //Backward(10,40);
  //PivotLeft(90,40);
  //PivotRight(90,40);

  //leftServo.writeMicroseconds(pulse);
  //for loop creates 30 second delay
  for(sec=0; sec<30; sec++)
  {
    delay(1000);
  }
  //terminate signals to servos
  //leftServo.detach();
  //rightServo.detach();
}

//into M1
void loop() {
  // put your main code here, to run repeatedly:
  int line; //used to define the IR reading of the line (if line is black or white change this value => black here)
  int offLine; //used to define output value for the background off the line (white or black => white for line follower)
  float A, B, C, D, E, F; //output values from the sensor //make int?,

  A = analogRead(A1); //A1 to A5 for the pins
  B = analogRead(A2); //A1 to A5 for the pins
  C = analogRead(A3); //A1 to A5 for the pins
  D = analogRead(A4); //A1 to A5 for the pins
  E = analogRead(A5); //A1 to A5 for the pins
  A = analogRead(A1); //A1 to A5 for the pins

  //statements to check if on the line (conditionals), instructions on what to do based on the readings


  //set the speed of the motor: goes from 0 to 255 (set an if statement to error check if within range)
  Motor1.setSpeed(100);

  Motor1.run(FORWARD);
  delay(1000);

  Motor1.run(BACKWARD);
  delay(1000);

  Motor1.run(RELEASE);
  delay(1000);

}

void leftMotorRPM(int RPM)
{
  int pulse;
  pulse = RPM * 80/40 + 1500;
  // leftServo.writeMicroseconds(pulse);
}

void rightMotorRPM(int RPM)
{
  int pulse;
  pulse = RPM * 80/40 + 1500;
  //rightServo.writeMicroseconds(pulse);
}

void Forward( int Dist , int Speed){
  long int Time = (long int)Dist*2900 / (Speed);

   Serial.print("Forward ");
   Serial.println(Time);

   rightMotorRPM(Speed);
   leftMotorRPM(-Speed);

   delay(Time);

   rightMotorRPM(0);
   leftMotorRPM(0);

}

void Backward( int Dist , int RPM){
   long int Time = (long int)Dist*2900 / (RPM);

   Serial.print("Backward ");
   Serial.println(Time);

   rightMotorRPM(-RPM);
   leftMotorRPM(RPM);

   delay(Time);

   rightMotorRPM(0);
   leftMotorRPM(0);
}

void PivotLeft( int Angle , int RPM){
 long int Time = (long int)Angle*290/RPM;

  rightMotorRPM(-RPM);
  leftMotorRPM(-RPM);

  delay(Time);

  rightMotorRPM(0);
  leftMotorRPM(0);
}

void PivotRight( int Angle , int RPM){
   long int Time = (long int)Angle*290/RPM;

  rightMotorRPM(RPM);
  leftMotorRPM(RPM);

  delay(Time);

  rightMotorRPM(0);
  leftMotorRPM(0);
}
