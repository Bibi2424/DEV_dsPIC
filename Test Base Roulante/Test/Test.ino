
#define MARCHE_AVANT   1
#define MARCHE_ARRIERE 3
#define ARRET          2

#define VEILLE  0
#define SENS    83
#define VITESSE 86
#define RETOUR_LIGNE 10

int led = 13;
int in1 = 2;
int in2 = 3;
int in3 = 4;
int in4 = 5;
int pwm1 = 10;
int pwm2 = 11;

int data = 0;
int state = VEILLE;
int vitesse = 0;

// the setup routine runs once when you press reset:
void setup() 
{                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT); 
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(pwm1, OUTPUT);  
  pinMode(pwm2, OUTPUT);
  // initialize serial:
  Serial.begin(9600);
  
  choix_sens(ARRET);
  
  choix_vitesse(0);
}

// the loop routine runs over and over again forever:
void loop() 
{  
  parse_data();
  delay(1000);
}

void parse_data()
{
  data = Serial.read();
  if(data == -1) return;
  switch (data) {
    case VEILLE:
    break;
    case SENS:
      data = Serial.read();
      choix_sens(data-48);
      while(Serial.read() != 10);
      state = VEILLE;
    break;
    case VITESSE:
      data = Serial.read();
      while(data != 10) {
        vitesse = 10*vitesse + (data-48);
        data = Serial.read();
      }
      choix_vitesse(vitesse);
      vitesse = 0;
      state = VEILLE;
    break;
    case RETOUR_LIGNE:
    break;
    default:
      Serial.println("Commande invalide");
  }
  return;
}

void choix_sens(char sens)
{
  if (sens == MARCHE_AVANT) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Serial.println("Marche avant");
  }
  else if (sens == MARCHE_ARRIERE) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Serial.println("Marche arriere");
  }
  else if (sens == ARRET) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    Serial.println("Arret");
  }
  return;
}

void choix_vitesse(int vitesse)
{
  analogWrite(pwm1, vitesse);
  analogWrite(pwm2, vitesse);
  Serial.print("Vitesse = ");
  Serial.println(vitesse);
  return;
}
