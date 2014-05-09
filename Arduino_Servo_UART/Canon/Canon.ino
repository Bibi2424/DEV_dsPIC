// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 

Servo myservo;  // create servo object to control a servo 
// a maximum of eight servo objects can be created 
int button_pin = 2;
int relais_pin = 12;

int in = 0;
int in2 = 0;

void setup() 
{ 
  myservo.attach(5);  // attaches the servo on pin 5 to the servo object 
  myservo.write(90);
  Serial.begin(9600);
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(relais_pin, OUTPUT);
} 


void loop() 
{ 
  in = Serial.read();
  in2 = digitalRead(button_pin);
  if(in2 == 0) {
    Serial.println("Fire !!!");
    myservo.write(15);
    delay(1000);
    myservo.write(100);
  }
  if(in == 71 || in == 103) {
    digitalWrite(relais_pin, HIGH);
    Serial.println("Moteur ON");
  }
  else if(in == 83 || in == 115) {
    digitalWrite(relais_pin, LOW);
    Serial.println("Moteur OFF");
  }
} 

