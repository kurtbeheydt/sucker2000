boolean sensorOnEdge (uint8_t pin);
void moveWheels(uint32_t velocity, uint8_t dir = HIGH);
void turnWheels(uint8_t dir = HIGH, uint32_t velocity = 150);

#define pinDirectionA 12 // A = linkse wiel
#define pinDirectionB 13
#define pinSpeedA 3
#define pinSpeedB 11
#define lefEdgeSensor 4             
#define rightEdgeSensor 5              
#define edgeSensorThreshold 1000  

#define ACTION_MOVE 0
#define ACTION_TURN 1

#define TURN_LEFT LOW
#define TURN_RIGHT HIGH
#define TURN_DURATION 2000000 // 2 secs


uint32_t turnStart;
byte action;
byte prevAction;
byte turnDirection;

void setup()
{
  delay(500);
  pinMode(pinDirectionA, OUTPUT); 
  pinMode(pinDirectionB, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  
  action = ACTION_MOVE;
  prevAction = ACTION_MOVE;
}


void loop()
{
  // check what to do
  if (sensorOnEdge(lefEdgeSensor)) {
     Serial.println("LEFT OVER EDGE");
     action = ACTION_TURN;
     turnDirection = TURN_RIGHT;
  } else if (sensorOnEdge(rightEdgeSensor)) {
     Serial.println("RIGHT OVER EDGE");
      action = ACTION_TURN;
      turnDirection = TURN_LEFT;
  }
  
  // start or continue the action
  switch (action) {
    
    case ACTION_MOVE:
      Serial.println("Moving");
      moveWheels(100, LOW);
      break;
      
    case ACTION_TURN:
      // init in case of start turning
      if (prevAction == ACTION_MOVE) {
        Serial.println("Start turning");
        stopWheels();
        turnStart = micros();
      }
      // turn
      turnWheels(turnDirection);
      if (turnDirection == TURN_LEFT) {
        Serial.println("Turning left");
      } else {
        Serial.println("Turning right");
      }
      
      // check if we can stop turning
      uint32_t divTurnTime = micros() - turnStart;
      //Serial.print("Turned time ");
      //Serial.println(divTurnTime, DEC);
      if (divTurnTime > TURN_DURATION) {
        Serial.println("Stop turning");
        action = ACTION_MOVE;
      }
      break;
      
  }
  prevAction = action;
 
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

// returns true if the sensor is over the edge (for the given edge pin)

boolean sensorOnEdge(uint8_t pin) 
{
  pinMode(pin, OUTPUT);  
  digitalWrite(pin, HIGH);
  uint32_t t0 = micros();
  uint32_t dif;
  pinMode(pin, INPUT);
  
  while(digitalRead(pin)) {
    dif = micros() - t0;
    if (dif > edgeSensorThreshold) break;
  }
  return dif > edgeSensorThreshold;

}
