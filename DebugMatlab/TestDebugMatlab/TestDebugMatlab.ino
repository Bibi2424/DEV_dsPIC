
double i = 0.0, f=0.0;
int run = 0;

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  delay(1000);
  
  //Serial.println("Goodnight moon!");
}

void loop() // run over and over
{
  while(Serial.available() <= 0 && run == 0)
      run = Serial.read();
      Serial.flush();
  while(1)
  {
      if(Serial.available() > 0)
          run = Serial.read();
          Serial.flush();
      if(run == 1)
      {
          f = 1.1*sin(i);
          i += 0.1;
        //Serial.print("sin de ");
          Serial.print(i);
          Serial.print(" ");
          Serial.println(f);
          delay(10);
      }
      else if(run == 2)
      {
          i = 0;
          run = 0;
      }
  }
}

