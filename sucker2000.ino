//#include <Servo.h>

boolean sensorOnEdge (uint8_t pin);
void moveWheels(uint32_t velocity, uint8_t dir = HIGH);
void turnWheels(uint8_t dir = HIGH, uint32_t velocity = 180);

#define pinDirectionA 12 // A = linkse wiel
#define pinDirectionB 13
#define pinSpeedA 3
#define pinSpeedB 11
#define rightEdgeSensor 4             
#define leftEdgeSensor 5              
#define edgeSensorThreshold 130  

#define ACTION_MOVE 0
#define ACTION_TURN 1

#define TURN_LEFT LOW
#define TURN_RIGHT HIGH
#define TURN_DURATION 1500000 // 2 secs


uint32_t turnStart;
byte action;
byte prevAction;
byte turnDirection;

/*
Servo sensorServo;

#define pinSensorservo 6
int servoAngle = 90;
uint8_t servoDirection = true;
uint32_t servoInterval = 10;
*/

uint32_t timepast = 0;

void setup()
{
  pinMode(pinDirectionA, OUTPUT); 
  pinMode(pinDirectionB, OUTPUT);
  //sensorServo.attach(pinSensorservo);
  //sensorServo.write(servoAngle);
  
  Serial.begin(9600);
  delay(1000);
  
  action = ACTION_MOVE;
  prevAction = ACTION_MOVE;
}


void loop()
{
  //turnSensor();
  

  // check what to do
  if (sensorOnEdge(rightEdgeSensor)) {
     Serial.println("RIGHT OVER EDGE");
     action = ACTION_TURN;
     turnDirection = TURN_RIGHT;
  }
  if (sensorOnEdge(leftEdgeSensor)) {
     Serial.println("LEFT OVER EDGE");
      action = ACTION_TURN;
      turnDirection = TURN_LEFT;
  }
  //delay(1000);
  
  // start or continue the action
  switch (action) {
    
    case ACTION_MOVE:
      Serial.println("Moving");
      moveWheels(120, HIGH);
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

/*
void turnSensor()
{
  if (millis() >= (timepast + servoInterval)) {
    timepast = millis();
    servoAngle = (servoDirection) ? (servoAngle + 1) : (servoAngle - 1);
    if (servoAngle >= 130) {
      servoDirection = false;
    }
    if (servoAngle <= 50) {
      servoDirection = true; 
    }
    sensorServo.write(servoAngle);
    //Serial.println(timepast);
  }  
}
*/

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
  //delay(50);
}

void turnWheels(uint8_t dir, uint32_t velocity)
{
  stopWheels();
  if (dir == 0) {
    digitalWrite(pinDirectionA, HIGH);
    digitalWrite(pinDirectionB, LOW);
  } else {
    digitalWrite(pinDirectionA, LOW);
    digitalWrite(pinDirectionB, HIGH);
  }
  analogWrite(pinSpeedA, velocity);
  analogWrite(pinSpeedB, velocity);
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
  
  Serial.print(pin);
  Serial.print(": ");
  Serial.println(dif);
  return dif > edgeSensorThreshold;

}
