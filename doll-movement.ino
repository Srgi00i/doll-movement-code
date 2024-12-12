#include <AccelStepper.h>
#include <Encoder.h>
#include <Wire.h>

// Pin definitions
#define EN_PIN 7
#define DIR_PIN 8
#define STEP_PIN 9

#define HALL_PIN 10
#define BUTTON_A_PIN 11
#define BUTTON_B_PIN 12

#define STEPPER_SPEED 200 // Velocidad maxima del stepper
#define STEPPER_ACCEL 100 // Aceleración del stepper
#define STEPPER_ENABLE LOW // Tipo de enable (alto/bajo)

enum State
{
  INIT,   // 0 by default
  MANUAL, // 1
  AUTO    // 2
};

enum Code
{
  BUTTON_A,
  BUTTON_B
};

enum Dir
{
  FRONT,
  BACK
};

// Initialize stepper motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Motor parameters
const int stepsPerRevolution = 1600; // Adjust based on your motor's specs
int halfRotation = stepsPerRevolution / 2;

// Sensor parameters

// Logic parameters
State mode = INIT;
Dir headDir = FRONT;
int loopDelay = 0;
unsigned long previousMillis = 0;
const long interval = 20 * 1000; // 20 seconds

void setup()
{
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Power on:");

  pinMode(EN_PIN, OUTPUT);
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(BUTTON_A_PIN, INPUT);
  pinMode(BUTTON_B_PIN, INPUT);

  // Initialize stepper motor parameters
  digitalWrite(EN_PIN, STEPPER_ACCEL);
  //stepper.setMaxSpeed(200);     // Set max speed
  //stepper.setAcceleration(100); // Set acceleration

  mode = INIT;

  homing();

  mode = AUTO;

  // stepper.moveTo(50);
}

bool checkHoming()
{
  if (digitalRead(HALL_PIN) == LOW)
  { 
    // Check if button is pressed (LOW due to pull-up)
    return true;
  }
  else
  {
    return false;
  }
}

void homing()
{
  digitalWrite(EN_PIN, STEPPER_ENABLE);
  stepper.setMaxSpeed(400);     // Set max speed
  stepper.setAcceleration(200); // Set acceleration

  Serial.println("homing...");

  while (!checkHoming())
  {
    stepper.moveTo(stepper.currentPosition() + 10);
    stepper.run();
  }

  Serial.println("done!");
}

void goTo(Dir dir)
{
  Serial.println("Turning...");

  digitalWrite(EN_PIN, LOW);
  stepper.setMaxSpeed(200);     // Set max speed
  stepper.setAcceleration(100); // Set acceleration
  
  if (dir == BACK)
  {
    stepper.moveTo(stepper.currentPosition() + halfRotation);
    while (stepper.distanceToGo() != 0)
    {
      stepper.run();
    }

    headDir = BACK;
  }
  else if (dir == FRONT)
  {
    stepper.moveTo(stepper.currentPosition() - halfRotation);
    while (stepper.distanceToGo() != 0)
    {
      stepper.run();
    }

    headDir = FRONT;
  }
  
}

void buttonFunctionA()
{
  Serial.println("Executing function A...");

  digitalWrite(EN_PIN, STEPPER_ENABLE);
  stepper.setMaxSpeed(STEPPER_SPEED);     // Set max speed
  stepper.setAcceleration(STEPPER_ACCEL); // Set acceleration

  if (mode == AUTO)
  {
    Serial.println("Switching mode -> manual");
    mode = MANUAL;
  }

  if (headDir == FRONT)
  {
    goTo(BACK);
  }
  else
  {
    goTo(FRONT);
  }
}

void buttonFunctionB()
{
  Serial.println("Executing function B...");

  digitalWrite(EN_PIN, STEPPER_ENABLE);
  stepper.setMaxSpeed(STEPPER_SPEED);     // Set max speed
  stepper.setAcceleration(STEPPER_ACCEL); // Set acceleration

  if (mode == MANUAL)
  {
    Serial.println("Switching mode -> auto");
    mode = AUTO;

    if (headDir == BACK)
    {
      goTo(FRONT);
    }
  }
}

void autoFunction()
{
  Serial.println("Executing auto function...");

  Serial.println("Executing function A...");

  digitalWrite(EN_PIN, STEPPER_ENABLE);
  stepper.setMaxSpeed(STEPPER_SPEED);     // Set max speed
  stepper.setAcceleration(STEPPER_ACCEL); // Set acceleration

  if (headDir == FRONT)
  {
    goTo(BACK);
    delay(1000);
    goTo(FRONT);
  }
  else {
    goTo(FRONT);
  }
}

void loop()
{
  if (digitalRead(BUTTON_A_PIN) == HIGH)
  {
    buttonFunctionA();
  }
  if (digitalRead(BUTTON_B_PIN) == HIGH)
  {
    buttonFunctionB();
  }

  unsigned long currentMillis = millis();

  if (mode == AUTO)
  {
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      autoFunction();
    }
  }

  delay(loopDelay);
}
