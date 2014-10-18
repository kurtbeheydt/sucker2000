boolean sensorOnEdge (uint8_t pin);
void moveWheels(uint8_t dir = HIGH);
void turnWheels(uint8_t dir = HIGH, uint32_t velocity = 180);

#define pinDirectionA 12 // A = linkse wiel
#define pinDirectionB 13
#define pinSpeedA 3
#define pinSpeedB 11
#define pinRightEdgeSensor 4             
#define pinLeftEdgeSensor 5
#define pinBeacon 2
#define pinPiezo 8
#define pinPowerTrigger 6
#define pinPowerEcho 7
#define pinSpeedPotentio 3

#define edgeSensorThreshold 130  

#define ACTION_STANDBY 0
#define ACTION_STARTING 1
#define ACTION_MOVE 2
#define ACTION_TURN 3
#define ACTION_STOPPING 4

#define TURN_LEFT LOW
#define TURN_RIGHT HIGH
#define TURN_DURATION 1500000 // 2 secs
uint32_t turnStart;

byte action;
byte prevAction;
byte turnDirection;
byte beaconActive = false;
uint32_t beaconTimePast = 0;

long powerDuration;
uint32_t powerTimePast = 0;

uint32_t startupTimePast = 0;
#define STARTUP_DURATION 3500 // 3 secs

void setup()
{
  pinMode(pinBeacon, OUTPUT); 
  pinMode(pinDirectionA, OUTPUT); 
  pinMode(pinDirectionB, OUTPUT);
  pinMode(pinPowerTrigger, OUTPUT);
  pinMode(pinPowerEcho, INPUT);
  
  Serial.begin(9600);
  delay(1000);
  
  action = ACTION_STANDBY;
  prevAction = ACTION_MOVE;
}


void loop()
{
  // zwaailicht
  blinkBeacon();
  
  // detect Start/Stop
  readPowerSensor();
  
  if ((action == ACTION_MOVE) || (action == ACTION_TURN)) {
    // check for edges)
    if (sensorOnEdge(pinRightEdgeSensor)) {
       Serial.println("RIGHT OVER EDGE");
       action = ACTION_TURN;
       turnDirection = TURN_RIGHT;
    }
    if (sensorOnEdge(pinLeftEdgeSensor)) {
       Serial.println("LEFT OVER EDGE");
        action = ACTION_TURN;
        turnDirection = TURN_LEFT;
    }
  }
  
  // start or continue the action
  switch (action) {
    case ACTION_STANDBY:
      Serial.println("Waiting to start");
      break;
      
    case ACTION_STARTING:
      Serial.println("Starting");
      if (millis() > (startupTimePast + STARTUP_DURATION)) {
        action = ACTION_MOVE;
      }
      break;
            
    case ACTION_STOPPING:
      Serial.println("Stopping");
      if (millis() > (startupTimePast + STARTUP_DURATION)) {
        action = ACTION_STANDBY;
      }
      break;
      
    case ACTION_MOVE:
      Serial.println("Moving");
      moveWheels();
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

void moveWheels(uint8_t dir)
{
  stopWheels();
  
  uint32_t velocity = analogRead(pinSpeedPotentio);
  Serial.println(velocity);
  velocity = map(velocity, 0, 1014, 50, 150);
  
  Serial.println(velocity);
  digitalWrite(pinDirectionA, dir);
  digitalWrite(pinDirectionB, dir);
  analogWrite(pinSpeedA, velocity);
  analogWrite(pinSpeedB, velocity);
}

void stopWheels()
{
  analogWrite(pinSpeedA, 0);
  analogWrite(pinSpeedB, 0);
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

/**
* returns true if the sensor is over the edge (for the given edge pin)
*/
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

void blinkBeacon()
{
  if (millis() > (beaconTimePast + 500)) {
    noTone(pinPiezo);
    beaconTimePast = millis();
    if ((action == ACTION_STARTING) || (action == ACTION_STOPPING)) {
      digitalWrite(pinBeacon, HIGH);
      if (beaconActive) {
        beaconActive = false;
      } else {
          tone(pinPiezo, 2000);
        beaconActive = true;
      }
    } else {
      if (beaconActive) {
        digitalWrite(pinBeacon, LOW);
        beaconActive = false;
      } else {
        digitalWrite(pinBeacon, HIGH);
        if ((action == ACTION_STARTING) || (action == ACTION_STOPPING)) {
          tone(pinPiezo, 2000);
        }
        beaconActive = true;
      }
    }
  }
}

void readPowerSensor() {
  if (millis() > (powerTimePast + 500)) {
    powerTimePast = millis();
    digitalWrite(pinPowerTrigger, LOW);
    delayMicroseconds(2);
    digitalWrite(pinPowerTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinPowerTrigger, LOW);
    powerDuration = pulseIn(pinPowerEcho, HIGH);
    
    //Serial.println(powerDuration);
  
    if (powerDuration < 1000) {
      // to avoid an immidiate repetitive detection
      powerTimePast = millis() + 1000;
      startupTimePast = millis();
      if (action == ACTION_STANDBY) {
        action = ACTION_STARTING;
      } else {
        action = ACTION_STOPPING;
        stopWheels();
      }
    }
  }
}
