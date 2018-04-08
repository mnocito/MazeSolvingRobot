#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
float firstHeading = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
NewPing sonars[3] = {
  NewPing(4, 5, 125), NewPing(9, 10, 125), NewPing(6, 7, 125)
};
unsigned long startTime = 0;
unsigned long pingTimer = 0;

//direction assignment
#define FWD  0
#define RVS 1
#define LEFT 0
#define FORWARD 1
#define RIGHT 2
#define BACK -1

// Motor definitions to make life easier:
#define MOTOR_A 0
#define MOTOR_B 1

// Pin Assignments //
//Default pins:
#define DIRA 2 // Direction control for motor A
#define PWMA 3  // PWM control (speed) for motor A
#define DIRB 4 // Direction control for motor B
#define PWMB 11 // PWM control (speed) for motor B

// other assignments
#define DRIVE_FORWARD_TIME 720
unsigned long driveForwardTime = 0;
bool turning = false;
int lastTurn = -1;

void setup(void)
{
  startTime = millis() + 30;
  pingTimer = millis() + 150;
  setupArdumoto(); // Set all pins as outputs
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1500);
  bno.setExtCrystalUse(true);
}

void loop(void) {
  sensors_event_t event;
  bno.getEvent(&event);
  if (canGo(LEFT) && lastTurn != LEFT) {
    lastTurn = LEFT;
    Serial.println("can go left");
    turnDir(RIGHT);
  } else if (canGo(FORWARD)) {
    Serial.println("can go forward");
    driveForward();
  } else if (canGo(RIGHT) && lastTurn != RIGHT) {
    lastTurn = RIGHT;
    Serial.println("can go right");
    turnDir(RIGHT);
  } else {
    lastTurn = BACK;
    Serial.println("can go back");
    turnDir(BACK);
  }
  delay(100);
}


void gyroCorrect(float eventHeading) {
  while (eventHeading > 3.0 && eventHeading < 355.0) {
    sensors_event_t event;
    bno.getEvent(&event);
    eventHeading = event.orientation.x;
    if (eventHeading > 180) {
      if (millis() > pingTimer && millis() < pingTimer + 100) {
        stopArdumoto(MOTOR_A);
        stopArdumoto(MOTOR_B);
      } else if (millis() > pingTimer + 100 && millis() < pingTimer + 200) {
        driveArdumoto(MOTOR_A, RVS, 33);
        driveArdumoto(MOTOR_B, FWD, 33);
      } else if (millis() > pingTimer + 200) {
        pingTimer = millis() + 200;
      }
    } else {
      if (millis() > pingTimer && millis() < pingTimer + 150) {
        stopArdumoto(MOTOR_A);
        stopArdumoto(MOTOR_B);
      } else if (millis() > pingTimer + 150 && millis() < pingTimer + 200) {
        driveArdumoto(MOTOR_A, FWD, 33);
        driveArdumoto(MOTOR_B, RVS, 33);
      } else if (millis() > pingTimer + 200) {
        pingTimer = millis() + 200;
      }
    }
  }
  delay(250);
}


void moveBack() {
  unsigned long backupTime = millis() + 150;
  while (millis() < backupTime) {
    driveArdumoto(MOTOR_A, RVS, 40);
    driveArdumoto(MOTOR_B, RVS, 40);
  }
  stopArdumoto(MOTOR_A);
  stopArdumoto(MOTOR_B);
}

void moveForward() {
  unsigned long forwardTime = millis() + 300;
  while(millis() < forwardTime) {
    driveArdumoto(MOTOR_A, FWD, 48);
    driveArdumoto(MOTOR_B, FWD, 50);
  }
  stopArdumoto(MOTOR_A);
  stopArdumoto(MOTOR_B);
}

void driveForward() {
  sensors_event_t event;
  bno.getEvent(&event);
  float eventHeading = event.orientation.x;
  Serial.println("event roien");
  Serial.println(event.orientation.x);
  int firstReading = sonars[1].ping_cm();
  int currentLeftReading = 0;
  int currentRightReading = 0;
  int currentReading = 0;

  while (true) {
    sensors_event_t event;
    bno.getEvent(&event);
    eventHeading = event.orientation.x;
    Serial.print("HEADING BEFORE CHECK");
    Serial.println(eventHeading);
    gyroCorrect(eventHeading);
    Serial.print("first reading: ");
    Serial.println(firstReading);
    currentReading = sonars[1].ping_cm();
    Serial.print("current reading: ");
    Serial.println(currentReading);
    Serial.print("headiL");
    Serial.println(event.orientation.x);
    if (sonars[1].ping_cm() <= 3) {
      moveBack();
      stopArdumoto(MOTOR_A);
      stopArdumoto(MOTOR_B);
      eventHeading = event.orientation.x;
      Serial.print("HEADING BEFORE CHECK");
      Serial.println(eventHeading);
      gyroCorrect(event.orientation.x);
      break;
    } else if (firstReading - currentReading == 0 || firstReading - currentReading <= 17) {
      driveArdumoto(MOTOR_A, FWD, 46);
      driveArdumoto(MOTOR_B, FWD, 46);
    } else if (firstReading - currentReading >= 19) {
      driveArdumoto(MOTOR_A, RVS, 46);
      driveArdumoto(MOTOR_B, RVS, 46);
    } else {
      stopArdumoto(MOTOR_A);
      stopArdumoto(MOTOR_B);
      break;

    }
  }
  delay(500);
}

boolean canGo(int dir) {
  return (sonars[dir].ping_cm() / 13) != 0;
}

void turnDir(int dir) {
  float eventHeading = 0;
  turning = true;
  bno.begin();
  switch (dir) {
    case LEFT:
      delay(500);
      while (eventHeading <= 265 || eventHeading > 355) {
        sensors_event_t event;
        bno.getEvent(&event);
        eventHeading = event.orientation.x;
        if (millis() > pingTimer && millis() < pingTimer + 100) {
          stopArdumoto(MOTOR_A);
          stopArdumoto(MOTOR_B);
        } else if (millis() > pingTimer + 100 && millis() < pingTimer + 200) {
          driveArdumoto(MOTOR_A, RVS, 38);
          driveArdumoto(MOTOR_B, FWD, 38);
        } else if (millis() > pingTimer + 200) {
          pingTimer = millis() + 200;
        }
      }
      stopArdumoto(MOTOR_A);
      stopArdumoto(MOTOR_B);
      bno.begin();
      break;
    case RIGHT:
      delay(500);
      while (eventHeading <= 85 || eventHeading > 355) {
        sensors_event_t event;
        bno.getEvent(&event);
        eventHeading = event.orientation.x;
        if (millis() > pingTimer && millis() < pingTimer + 100) {
          stopArdumoto(MOTOR_A);
          stopArdumoto(MOTOR_B);
        } else if (millis() > pingTimer + 100 && millis() < pingTimer + 200) {
          driveArdumoto(MOTOR_A, RVS, 38);
          driveArdumoto(MOTOR_B, FWD, 38);
        } else if (millis() > pingTimer + 200) {
          pingTimer = millis() + 200;
        }
      }
      stopArdumoto(MOTOR_A);
      stopArdumoto(MOTOR_B);
      bno.begin();
      break;
    case BACK:
      delay(500);
      while (eventHeading <= 155) {
        sensors_event_t event;
        bno.getEvent(&event);
        eventHeading = event.orientation.x;
        if (millis() > pingTimer && millis() < pingTimer + 100) {
          stopArdumoto(MOTOR_A);
          stopArdumoto(MOTOR_B);
        } else if (millis() > pingTimer + 100 && millis() < pingTimer + 200) {
          driveArdumoto(MOTOR_A, RVS, 40);
          driveArdumoto(MOTOR_B, FWD, 40);
        } else if (millis() > pingTimer + 200) {
          pingTimer = millis() + 200;
        }
      }
      stopArdumoto(MOTOR_A);
      stopArdumoto(MOTOR_B);
      bno.begin();
      break;
    default:
      Serial.println("can't turn for some reason");
  }
} 

void driveArdumoto(byte motor, byte dir, byte spd) {
  if (motor == MOTOR_A) {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
  } else if (motor == MOTOR_B) {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }
}

// stopArdumoto makes a motor stop
void stopArdumoto(byte motor) {
  driveArdumoto(motor, 0, 0);
}

// setupArdumoto initialize all pins
void setupArdumoto() {
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}

float getHeading(int dir) {
  if (dir == LEFT) {
    return 360 - dir;
  } else if (dir == RIGHT) {
    return dir;
  }
}



