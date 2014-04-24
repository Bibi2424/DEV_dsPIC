   #include <Servo.h> 
   
  Servo myservo;  // Create servo object to control a servo 
                  // a maximum of eight servo objects can be created 
  int in = 0;     // Variable to read ASCII caracter
  int pos = 0;    // Variable to store the servo position 
   
  void setup() 
  { 
    myservo.attach(5);  // attaches the servo on pin 9 to the servo object 
    Serial.begin(9600); 
  } 
   
  void loop() 
  { 
    in = Serial.read();
    if(in != -1) 
    {
      while (in != 10)  
      {
        pos = pos*10 + (in-48);
        Serial.println(in);
        do
        {
          in = Serial.read();
        }while(in = -1);
      }
      myservo.write(pos);
      Serial.println(" ");
      Serial.println(pos);
      pos = 0;
    }
  } 
