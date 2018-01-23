
//source: http://www.goodliffe.org.uk/arduino/joystick.php
   /*
Read the Joystick. Try reading the analog values, and / or the digital value, to see what returns.
*/
//values are such that (9,6) [min] is the top left corner of the joystick
int analogInputPinX   = 1;
int analogInputPinY   = 0;
//int digitalInputPin  = 5;
int wait = 100; 
long analogInputValX;
long analogInputValY;
//int digitalInputVal
string TextOutput

void setup()
{
  pinMode(analogInputPinX,   INPUT);
  pinMode(analogInputPinY,   INPUT);
  //pinMode(digitalInputPin,   INPUT);
  
  Serial.begin(9600);  // ...set up the serial ouput on 0004 style
}

void loop()
{
  //converted input read to rough cartesian coordinates
  analogInputValX = analogRead(analogInputPinX)-542;
  analogInputValY = -1*analogRead(analogInputPinY)+525;
  //if condition: if between -25 and 25, analoginput = 0
  //digitalInputVal = digitalRead(digitalInputPin);
  
      //Serial.print("Digital is ");
      //Serial.print(digitalInputVal);
      Serial.print("Analog X is ");
      Serial.print(analogInputValX);    
      Serial.print(" and Analog Y is ");
      Serial.print(analogInputValY);    
      Serial.println(" "); // println, to end with a carriage return
   
  delay(wait); // Pause for 'wait' milliseconds before resuming the loop
}
