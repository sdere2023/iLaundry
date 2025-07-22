#include <Wire.h>
#include <Servo.h>

// Define the pins
const int IR_SENSOR_PIN = 2;  // Pin for the IR sensor
const int SERVO_PIN = 9;      // Pin for the servo motor
const int MOTOR_PIN_FORWARD = 3; // Pin for moving the robot forward
const int MOTOR_PIN_BACKWARD = 4; // Pin for moving the robot backward

Servo basketServo;  // Create a Servo object to control the basket

void setup() {
  // Set up the IR sensor pin
  pinMode(IR_SENSOR_PIN, INPUT);

  // Set up the servo motor
  basketServo.attach(SERVO_PIN);  // Attaches the servo on pin 9
  basketServo.write(0);  // Ensure the servo is in the initial position

  // Set up the motor pins for the wheels
  pinMode(MOTOR_PIN_FORWARD, OUTPUT);
  pinMode(MOTOR_PIN_BACKWARD, OUTPUT);

  // Start the serial monitor for debugging
  Serial.begin(9600);
}

void loop() {
  // Check the IR sensor
  int irSensorValue = digitalRead(IR_SENSOR_PIN);

  if (irSensorValue == LOW) {
    // If the IR sensor detects clothes
    Serial.println("Clothes detected!");

    // Step 1: Move the basket up
    raiseBasket();

    // Step 2: Move the robot to the washing machine
    moveRobotForward();

    // Step 3: Dump the clothes
    dumpClothes();

    // Step 4: Move the robot back to its original position
    moveRobotBackward();

    // Step 5: Lower the basket to prepare for more clothes
    lowerBasket();
  }
  delay(1000);  // Check the IR sensor every second
}

// Function to move the basket up
void raiseBasket() {
  Serial.println("Raising the basket...");
  basketServo.write(90);  // Adjust the angle to raise the basket
  delay(2000);  // Wait for 2 seconds to simulate raising the basket
}

// Function to dump the clothes
void dumpClothes() {
  Serial.println("Dumping clothes...");
  basketServo.write(180);  // Adjust the angle to dump the clothes
  delay(2000);  // Wait for 2 seconds to simulate dumping
}

// Function to lower the basket
void lowerBasket() {
  Serial.println("Lowering the basket...");
  basketServo.write(0);  // Move the servo back to its original position
  delay(2000);  // Wait for 2 seconds to simulate lowering the basket
}

// Function to move the robot forward
void moveRobotForward() {
  Serial.println("Moving forward to the washing machine...");
  digitalWrite(MOTOR_PIN_FORWARD, HIGH);  // Start moving forward
  delay(5000);  // Move forward for 5 seconds
  digitalWrite(MOTOR_PIN_FORWARD, LOW);   // Stop moving forward
}

// Function to move the robot backward
void moveRobotBackward() {
  Serial.println("Returning to the original position...");
  digitalWrite(MOTOR_PIN_BACKWARD, HIGH);  // Start moving backward
  delay(5000);  // Move backward for 5 seconds
  digitalWrite(MOTOR_PIN_BACKWARD, LOW);   // Stop moving backward
}
