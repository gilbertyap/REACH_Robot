// To-do
// Better serial error catching
// Make photon do less of the work, put more work in python
// Motion Smoothing (better intervals)
// Add a physical button to turn on and off wifi connect?

// Future
// Shoulder, wrist, arm movement
// RISE Poster
// PI 3 for portable python station

SYSTEM_MODE(SEMI_AUTOMATIC) // disconnect from wifi, turn on later
#include "math.h"

// Set up IO
Servo baseServo; // Bottom, base rotation
Servo reachServo; // Forward and backwards of arm
Servo heightServo; // Z-axis moving of arm
Servo clawServo; // Claw servo

/**********
--Maximum Angles of Servos--
Base Servo - 0 = right most, 180 = left most
Reach Servo - 60 = vertical postion, 145 = forward most
Height Servo - 0 = lowest height, 120 = highest height (about 90 deg motion)
Claw Servo - 0 = closed, ~<90 = past widest open
***********/

int trackedPosB = 0;
int trackedPosR = 0;
int trackedPosH = 0;
int trackedPosC = 0;

int moveToB = 0;
int moveToR = 0;
int moveToH = 0;
int moveToC = 0;

int readBuffer;

void setup() {
  baseServo.attach(D0); // Attach servos to PWM IO
  heightServo.attach(D1);
  clawServo.attach(D2);
  reachServo.attach(D3);
  //attachInterrupt(D6, connect, RISING);

  // Overflow of serial data is overwritten
  Serial.blockOnOverrun(false);
  Serial.begin(19200);

  RGB.control(true);

  RGB.color(255,0,0);
  calibrate();
  RGB.color(0,255,0);
}

void loop() {
  // Checks that 10 values are in queue, (10 steps required for 1 movement)
  while (Serial.available() > 10) {
    setMoves();
  }
}
/*
// Connects to wifi if D6 is brought high
void connect() {
  if (Particle.connected() == false) {
    Particle.connect();
  }
}
*/
// Reads every serial event and checks for the shut down event
void serialEvent() {
  if(Serial.peek() == 254) {
    multiStepTo(90, 90, 100, 85);
    delay(1000);
    Serial.end();
  }
}

// Reads for different serial data and "moveTo" values then executes
void setMoves() {
  RGB.color(255,0,255);

  readBuffer = Serial.read(); // read first value

  if(readBuffer == 240) { // Start indicator
    readBuffer = Serial.read(); // read the next value

    if(readBuffer == 241) { // Indicator for X value
      moveToB = Serial.read(); // read again since value that follows is X
      readBuffer = Serial.read();
    }

    if (readBuffer == 242) { // Indicator for Y Value
      moveToH = Serial.read(); // read for H
      readBuffer = Serial.read();
    }

    if(readBuffer == 243) { // Indicator for Z value "reach"
      moveToR = Serial.read(); // read for R
      readBuffer = Serial.read();
    }

    if(readBuffer == 244) { // Indicator for Claw`
      moveToC = Serial.read(); // read for C
      readBuffer = Serial.read();
    }
  }

  if (readBuffer == 245) { // end of all coordinates
    // Indicate moving phase on RGB LED
    RGB.color(0,0,255);

    // Move motors
    multiStepTo(moveToB, moveToR, moveToH, moveToC);
  }
}

// Set initial arm position
void calibrate() {
  stepTo('b', 0, 90, 1);
  delay(500);
  stepTo('h', 45, 100, 1); // set height first because it keeps hitting itself
  delay(500);
  stepTo('r', 45, 90, 1);
  delay(500);
  stepTo('c', 0, 85, 1);
  delay(2000);
}

// Used for calibrate to set initial trackedPos positions
void stepTo(char servo, int start, int angle, int speed) {
  if(start >= angle) {
    for(int i = start; i >= angle; i-=speed) {
      switch(servo) {
        case 'b':
        baseServo.write(i);
        trackedPosB = angle;
        break;
        case 'r':
        reachServo.write(i);
        trackedPosR = angle;
        break;
        case 'h': // vertical height
        heightServo.write(i);
        trackedPosH = angle;
        break;
        case 'c': // hand
        clawServo.write(i);
        trackedPosC = angle;
        break;
      }
      delay(10);
    }
  } else {
    for(int i = start; i <= angle; i+=speed) {
      switch(servo) {
        case 'b':
        baseServo.write(i);
        trackedPosB = angle;
        break;
        case 'r':
        reachServo.write(i);
        trackedPosR = angle;
        break;
        case 'h': // vertical height
        heightServo.write(i);
        trackedPosH = angle;
        break;
        case 'c': // hand
        clawServo.write(i);
        trackedPosC = angle;
        break;
      }
      delay(10);
    }
  }
}

void multiStepTo(int bAngle, int rAngle, int hAngle, int cAngle) {
  // Find out which angle is the largest
  float maxDiff = findMax(bAngle, rAngle, hAngle, cAngle);

  Serial.println(maxDiff);

  // Make sure to divide every other motors' step by the number of the largest
  float bStepAngle = float(difference(bAngle,trackedPosB))/maxDiff;
  float rStepAngle = float(difference(rAngle, trackedPosR))/maxDiff;
  float hStepAngle = float(difference(hAngle, trackedPosH))/maxDiff;
  float cStepAngle = float(difference(cAngle, trackedPosC))/maxDiff;

  // Find out if step angle needs to be positive or negative
  if(bAngle < trackedPosB) {
    bStepAngle*=-1;
  }
  if(rAngle < trackedPosR) {
    rStepAngle*=-1;
  }
  if(hAngle < trackedPosH) {
    hStepAngle*=-1;
  }
  if(cAngle < trackedPosC) {
    cStepAngle*=-1;
  }

  Serial.println(bStepAngle);
  Serial.println(rStepAngle);
  Serial.println(hStepAngle);
  Serial.println(cStepAngle);

  // The servo with the most travel will move in increments of one
  // and the others will move proportionally to it

  // int + float will auto round to an int, round just in case?
  for(int step = 0; step < maxDiff; step++) {
      baseServo.write(trackedPosB+round(bStepAngle*step));
      reachServo.write(trackedPosR+round(rStepAngle*step));
      heightServo.write(trackedPosH+round(hStepAngle*step));
      clawServo.write(trackedPosC+round(cStepAngle*step));
    delay(3.0);
  }

  // Set the last known position as their current position after
  // the whole movement
  // Maybe want to make this at every movement
  trackedPosB = baseServo.read();
  trackedPosR = reachServo.read();
  trackedPosH = heightServo.read();
  trackedPosC = clawServo.read();
}

int findMax(int bAngle, int rAngle, int hAngle, int cAngle) {
  // List contains the delta of the angles
  int list[] = {difference(bAngle, trackedPosB),
    difference(rAngle, trackedPosR),
    difference(hAngle, trackedPosH),
    difference(cAngle, trackedPosC)};
  int hold;

  // Repeat this until all are sorted
  for(int i = 0; i <  arraySize(list)-1; i++) { // slot comparing to others
    for(int j = i+1; j < arraySize(list); j++) { // Compare-to Value in list
      if(list[i] > list[j]) {
        hold = list[j]; // Keep a copy of second item
        list[j] = list[i]; // Make next item first item
        list[i] = hold; // Replace first item with held value
      }
    }
  }
  return list[3]; // Return the right most value, most
}

int difference(int angle1, int angle2) {
  int diff = max(angle1, angle2) - min(angle1, angle2);
  return diff;
}
