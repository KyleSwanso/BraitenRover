//Included Libraries
#include <Wire.h>
#include <NewPing.h>
#include <TimerFreeTone.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//Pin definitions
#define triggerPinLeft 2
#define echoPinLeft 3
#define triggerPinMid 4
#define echoPinMid 5
#define triggerPinRight 6
#define echoPinRight 7
#define redPin 9
#define greenPin 10
#define bluePin 11
#define tonePin 12

//Global Variables.  Most settings can be changed here.
float readings[3];  //An array where we store our sensor readings.
enum heading {left, mid, right}; //Making it easy to address our array as readings[left], etc.
int maxSpeed = 255; //255 max.
int maxRange = 60; //Max sonar range in centimeters.


//MotorShield Objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);  //Left motor on motorshieldV2 port 4
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); //Right motor on motorshieldV2 port 1

//Sonar Objects
NewPing sonarLeft(triggerPinLeft, echoPinLeft, maxRange);
NewPing sonarMid(triggerPinMid, echoPinMid, maxRange);
NewPing sonarRight(triggerPinRight, echoPinRight, maxRange);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  AFMS.begin();
  
  //playMelody();
  playNote(698,250);
}


void loop() {
  // put your main code here, to run repeatedly:
  sensorSweep();
  
  int leftSpeed = 7 * ( readings[right] - 20 );
  int rightSpeed = 7 * ( readings[left] - 20 );

  if ( readings[mid] < 30){
    leftSpeed = -1 * maxSpeed;
    rightSpeed = -1 * maxSpeed;
  }
  else {
    diffDrive( leftSpeed, rightSpeed );
  }
  
  diffDrive( leftSpeed, rightSpeed );
  setRGB( (255 - readings[mid]), (255 - readings[left]), (255 - readings[right]) );
}

void sensorSweep () {
  //Read sensors and store readings in an array.
  //Serial.println("Sweep.");
  readings[left] = sonarLeft.ping_cm();
  delay(maxRange/4); //delay between pings is necessary.
  readings[mid] = sonarMid.ping_cm();
  delay(maxRange/4);
  readings[right] = sonarRight.ping_cm();
  delay(maxRange/4);
  
  if ( readings[left] == 0 ) {
    readings[left] = maxRange;
  }
  if ( readings[mid] == 0) {
    readings[mid] = maxRange;
  }
  if ( readings[right] == 0 ) {
    readings[right] = maxRange;
  }
  
  //Serial.println( readings[left] );
  //Serial.println( readings[mid] );
  //Serial.println( readings[right] );
}


void diffDrive (int speedLeft, int speedRight) {
  //Sets the speed of each motor. Differential drive.
  //Checks for negative value. (reverse)
  if (speedLeft < 0) {
    leftMotor->run(BACKWARD);
    speedLeft = abs(speedLeft);
  }
  else {
    leftMotor->run(FORWARD);
  }
  if (speedRight < 0) {
    rightMotor->run(BACKWARD);
    speedRight = abs(speedRight);
  }
  else {
    rightMotor->run(FORWARD);
  }
  
  if (speedLeft > maxSpeed)
    speedLeft = maxSpeed; //Limits speedLeft to maxSpeed.
  if (speedRight > maxSpeed)
    speedRight = maxSpeed; //Limits speedRight to maxSpeed.
    
  leftMotor->setSpeed(speedLeft);
  rightMotor->setSpeed(speedRight); 
}


void playNote (int frequency, int duration) {
  //Plays a single note based on frequency and duration.
  TimerFreeTone(tonePin, frequency, duration);
  delay(duration/20); //a small delay between notes.  
}


void playMelody() {
  //Plays a melody. Note and Duration stored in arrays.
  //This is a portion of Saria's song from Zelda:Ocarina of Time.
  int melody[] = { 698, 880, 988, 698, 880, 988, 698, 880, 988, 1319, 1175};
  int duration[] = { 250, 250, 500, 250, 250, 500, 250, 250, 250, 250, 500};
  
  for (int thisNote = 0; thisNote < 11; thisNote++) { // Loop through the notes in the array.
    setRGB(map(melody[thisNote], 125, 300, 0, 255), 0, map(duration[thisNote], 125, 300, 0, 255) ); //Set RGB for each note.  
    TimerFreeTone(tonePin, melody[thisNote], duration[thisNote]); // Play thisNote for duration.
    delay(50); // Short delay between notes.
  }
}


void setRGB (int R, int G, int B) {
  //Sets RGB LED.
  analogWrite(redPin, map( R, 0, 255, 0, 255 ) );  //map Value, From range, To range
  analogWrite(greenPin, map( G, 0, 255, 0, 255 ) );
  analogWrite(bluePin, map( B, 0, 255, 0, 255 ) );
}
