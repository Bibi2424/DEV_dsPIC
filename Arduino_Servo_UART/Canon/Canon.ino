 // Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
                
               
int in = 0;
 
void setup() 
{ 
  myservo.attach(5);  // attaches the servo on pin 9 to the servo object 
  myservo.write(90);
  Serial.begin(9600); 
} 
 
 
void loop() 
{ 
  in = Serial.read();
  if(in != -1) {
    myservo.write(15);
    delay(1000);
    myservo.write(90);
  }
} 
