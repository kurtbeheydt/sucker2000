uint32_t measure(uint8_t pin, uint32_t threshold = 3000);

void setup()
{
  delay(500);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
}


void loop()
{
  
  Serial.println(measure(3), DEC);
  //delay(500);
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
