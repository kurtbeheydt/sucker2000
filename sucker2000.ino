uint32_t measure(uint8_t pin, uint32_t threshold = 3000);
void moveWheels(uint32_t velocity, uint8_t dir = HIGH);
void turnWheels(uint8_t dir = HIGH, uint32_t velocity = 150);

#define pinDirectionA 12 // A = linkse wiel
#define pinDirectionB 13
#define pinSpeedA 3
#define pinSpeedB 11

void setup()
{
  delay(500);
  pinMode(pinDirectionA, OUTPUT); 
  pinMode(pinDirectionB, OUTPUT);
  Serial.begin(9600);
  delay(1000);
}


void loop()
{

// meten  
//  Serial.println(measure(3), DEC);

  moveWheels(100);
  delay(1500);
  
  stopWheels();
  delay(500);
  
  moveWheels(100, LOW);
  delay(1000);
  
  turnWheels();
  delay(1000);

  turnWheels(LOW);
  delay(1000);

}

uint32_t measure(uint8_t pin, uint32_t threshold) 
{
  pinMode(pin, OUTPUT);  
  digitalWrite(pin, HIGH);
  uint32_t t0 = micros();
  pinMode(pin, INPUT);
  
  while(digitalRead(pin)) {
    if ((micros() - t0) > threshold) {
      break;
    }
  }
  
  uint32_t t1 = micros();
  return t1 - t0;
}


void moveWheels(uint32_t velocity, uint8_t dir)
{
  stopWheels();
  digitalWrite(pinDirectionA, dir);
  digitalWrite(pinDirectionB, dir);
  analogWrite(pinSpeedA, velocity);
  analogWrite(pinSpeedB, velocity);
}

void stopWheels()
{
  analogWrite(pinSpeedA, 0);
  analogWrite(pinSpeedB, 0);
  delay(50);
}

void turnWheels(uint8_t dir, uint32_t velocity)
{
  stopWheels();
  if (dir == 1) {
    digitalWrite(pinDirectionA, HIGH);
    digitalWrite(pinDirectionB, LOW);
  } else {
    digitalWrite(pinDirectionA, LOW);
    digitalWrite(pinDirectionB, HIGH);
  }
  analogWrite(pinSpeedA, velocity);
  analogWrite(pinSpeedB, velocity);
  // XXX variabele tijd maken om de draaihoek te varieren
  delay(500);
}
