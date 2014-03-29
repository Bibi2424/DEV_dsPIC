
int led = 13;
int test = 0;

void setup() 
{
  // initialize serial:
  Serial.begin(9600);
  pinMode(led, OUTPUT);

}

void loop() 
{
  test = Serial.read();
  Serial.println(test);
  delay(1000);
}








