/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13. 
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead(). 
 
 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground
 
 * Note: because most Arduinos have a built-in LED attached 
 to pin 13 on the board, the LED is optional.
 
 
 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe
 
 This example code is in the public domain.
 
 http://arduino.cc/en/Tutorial/AnalogInput
 
 */

int X_pin = A1;
int Y_pin = A0;
int ledPin = 13;
int push_pin = 2;
int X_value = 0;
int Y_value = 0;
int push_value = 0;


void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT); 
  pinMode(push_pin, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("Good to go");
}

void loop() {
  // read the value from the sensor:
  X_value = analogRead(X_pin);  
  Y value = analogRead(Y_pin);
  Serial.print(X_value);
  Serial.print(" ");
  Serial.print(Y_value);
  Serial.print(" ");
  Serial.println(push_value);
}
