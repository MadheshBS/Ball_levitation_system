#include <Servo.h>

// --- Configuration ---
const int trigPin = 7;     // Changed from D1
const int echoPin = 6;     // Changed from D2
const int escPin = 9;      // Changed from D5 (PWM pin on Uno)
const float tubeLength = 55.0; // cm

// --- PID Gains ---
float Kp = 30.0;
float Ki = 3.0;
float Kd = 12.0;

// --- Variables ---
Servo esc;
float setpoint = 0;
bool systemActive = false;
float lastError = 0;
float integral = 0;
unsigned long lastTime;

void setup() {
  Serial.begin(9600);        // Changed from 115200 (Uno is stable at 9600)
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  esc.attach(escPin, 1000, 2000);
  Serial.println("\n--- 55cm Tube Ball Levitation ---");
  Serial.println("Arming ESC... Wait for beeps.");
  esc.writeMicroseconds(1000);
  delay(3000);

  Serial.println("READY. Enter height (0-55) or 100 to STOP.");
  lastTime = millis();
}

void loop() {
  // 1. Handle Serial Input
  if (Serial.available() > 0) {
    int input = Serial.parseInt();  // Changed to parseInt (more reliable on Uno)

    if (input == 100) {
      systemActive = false;
      integral = 0;
      esc.writeMicroseconds(1000);
      Serial.println("EMERGENCY STOP: Motor Off.");
    }
    else if (input >= 1 && input <= 55) {
      setpoint = (float)input;
      systemActive = true;
      integral = 0;
      lastError = 0;
      Serial.print("Target set to: "); Serial.print(setpoint); Serial.println(" cm");
    }
    else {
      Serial.println("Invalid. Enter 1-55 or 100 to stop.");
    }
    while (Serial.available() > 0) Serial.read(); // Clear buffer
  }

  // 2. Control Logic
  if (systemActive) {
    float rawDist = readUltrasonic();
    float currentHeight = tubeLength - rawDist;

    // PID Timing
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    if (dt < 0.005) dt = 0.005;   // Minimum 5ms guard

    // PID Math
    float error = setpoint - currentHeight;
    integral += error * dt;
    integral = constrain(integral, -50, 50);  // Anti-windup
    float derivative = (error - lastError) / dt;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    int baseThrottle = 1140;
    int throttle = constrain(baseThrottle + (int)output, 1000, 1800);

    esc.writeMicroseconds(throttle);

    // Save state
    lastError = error;
    lastTime = currentTime;

    // Debug output
    Serial.print("H: ");       Serial.print(currentHeight, 1);
    Serial.print(" | Sp: ");   Serial.print(setpoint, 1);
    Serial.print(" | Pwr: ");  Serial.println(throttle);

  } else {
    esc.writeMicroseconds(1000);
    lastTime = millis();
  }

  delay(30); // ~33Hz
}

float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  float cm = duration * 0.034 / 2.0;

  if (cm <= 0 || cm > 60) return 55.0; // Ball at bottom fallback
  return cm;
}