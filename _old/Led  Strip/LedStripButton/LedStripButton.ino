// color swirl! connect an RGB LED to the PWM pins as indicated
// in the #defines
// public domain, enjoy!
 
#define REDPIN   9
#define GREENPIN 6
#define BLUEPIN  3
#define BUTTON   12
#define LED      13
 
#define FADESPEED 100     // make this higher to slow down
#define INCR      10
#define BAL_SPEED 10
#define BAL_INCR  5
 
int r, g, b;
int pin;
int cpt, i;
 
void Balayage();
void Balayage2();
void Balayage3();
 
void setup() {
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  
  digitalWrite(REDPIN, LOW);
  digitalWrite(GREENPIN, LOW);
  digitalWrite(BLUEPIN, LOW);
  digitalWrite(LED, LOW);
  
  r = 0;
  g = 0;
  b = 0;
  cpt = 0;
}
 
 
void loop() {
 
  pin = digitalRead(BUTTON);
  if (pin == 0)
  {
    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
    cpt = (cpt+1)%3;
    i = 0;
  }

  switch (cpt)
  {
    case 0:
      Balayage();
    break;
    case 1:
      Balayage2();
    break;
    case 2:
      Balayage3();
    break;
  }
  
  analogWrite(REDPIN, r);
  analogWrite(GREENPIN, g);  
  analogWrite(BLUEPIN, b);
}

void RedPulse()
{
  r = (r + INCR)%256;
  g = 0;
  b = 0;
  delay(FADESPEED);
}

void GreenPulse()
{
  r = 0;
  g = (g + INCR)%256;
  b = 0;
  delay(FADESPEED);
}

void BluePulse()
{
  r = 0;
  g = 0;
  b = (b + INCR)%256;
  delay(FADESPEED);
}

void Balayage()
{
  switch(i)
  {
    case 0:
      r = r + BAL_INCR;
      g = 0;
      b = 0;
      if (r >= 255)
      {
        r = 255;
        i = 1;
      }
    break;
    case 1:
      r = r - BAL_INCR;
      g = 0;
      b = 0;
      if (r <= 0)
      {
        r = 0;
        i = 2;
      }
    break;
    case 2:
      r = 0;
      g = g + BAL_INCR;
      b = 0;
      if (g >= 255)
      {
        g = 255;
        i = 3;
      }
    break;
    case 3:
      r = 0;
      g = g - BAL_INCR;
      b = 0;
      if (g <= 0)
      {
        g = 0;
        i = 4;
      }
    break;
    case 4:
      r = 0;
      g = 0;
      b = b + BAL_INCR;
      if (b >= 255)
      {
        b = 255;
        i = 5;
      }
    break;
    case 5:
      r = 0;
      g = 0;
      b = b - BAL_INCR;
      if (b <= 0)
      {
        b = 0;
        i = 0;
      }
    break;
  }
  delay(BAL_SPEED);
}

void Balayage2()
{
  switch(i)
  {
    case 0:
      r = r + BAL_INCR;
      g = 0;
      b = 0;
      if (r >= 255)
      {
        r = 255;
        i = 1;
      }
    break;
    case 1:
      r = r - BAL_INCR;
      g = g + BAL_INCR;
      b = 0;
      if (r <= 0)
      {
        r = 0;
        g = 255;
        i = 2;
      }
    break;
    case 2:
      r = 0;
      g = g - BAL_INCR;
      b = b + BAL_INCR;
      if (g <= 0)
      {
        g = 0;
        b = 255;
        i = 3;
      }
    break;
    case 3:
      r = r + BAL_INCR;
      g = 0;
      b = b - BAL_INCR;
      if (b <= 0)
      {
        b = 0;
        r = 255;
        i = 1;
      }
    break;
  }
  delay(BAL_SPEED);
}

void Balayage3()
{
  switch(i)
  {
    case 0:
      r = r + BAL_INCR;
      g = 0;
      b = 0;
      if (r >= 255)
      {
        r = 255;
        i = 1;
      }
    break;
    case 1:
      r = 255;
      g = g + BAL_INCR;
      b = 0;
      if (g >= 255)
      {
        g = 255;
        i = 2;
      }
    break;
    case 2:
      r = 255;
      g = 255;
      b = b + BAL_INCR;
      if (b >= 255)
      {
        b = 255;
        i = 3;
      }
    break;
    case 3:
      r = r - BAL_INCR;
      g = 255;
      b = 255;
      if (r <= 0)
      {
        r = 0;
        i = 4;
      }
    break;
    case 4:
      r = 0;
      g = g - BAL_INCR;
      b = 255;
      if (g <= 0)
      {
        g = 0;
        i = 5;
      }
    break;
    case 5:
      r = 0;
      g = 0;
      b = b - BAL_INCR;
      if (b <= 0)
      {
        b = 0;
        i = 0;
      }
    break;
  }
  delay(BAL_SPEED);
}
