#include <PID_v1.h>
#include <Arduino.h>
class PIDController {
  private:
    float kp, ki, kd;      // PID coefficients
    float prevError;       // Previous error for derivative term
    float integral;        // Integral term
    float setpoint;        // Desired value
    float smoothingFactor; // Exponential Smoothing factor
    float filteredSpeed;   // Smoothed output for soft start
    
  public:
    PIDController(float p, float i, float d, float alpha, float initialSetpoint) {
      kp = p;
      ki = i;
      kd = d;
      smoothingFactor = alpha;
      prevError = 0;
      integral = 0;
      filteredSpeed = 0;
      setpoint = initialSetpoint; // Initialize setpoint
    }

    // Method to set the target speed
    void setSetpoint(float target) {
      setpoint = target;
    }

    // Exponential Smoothing Filter
    float smoothOutput(float newSpeed) {
      filteredSpeed = smoothingFactor * newSpeed + (1 - smoothingFactor) * filteredSpeed;
      return filteredSpeed;
    }

    // Calculate PID Output
    float compute(float currentSpeed, float dt) {
      float error = setpoint - currentSpeed;
      integral += error * dt;
      
      // Limit the integral term to prevent windup
      float integralLimit = 100.0; // Example limit value
      if (integral > integralLimit) integral = integralLimit;
      else if (integral < -integralLimit) integral = -integralLimit;
      
      float derivative = (error - prevError) / dt;
      
      // PID formula
      float output = kp * error + ki * integral + kd * derivative;
      
      prevError = error;

      // Smooth the output using Exponential Smoothing filter
      return smoothOutput(output);
    }
};

// Pin definitions
const int motorPin = 9; // PWM pin to control motor speed
const int speedSensorPin = A0; // Analog pin to read motor speed

// PID parameters
float kp = 2.0; // Proportional constant
float ki = 0.5; // Integral constant
float kd = 1.0; // Derivative constant

// Exponential Smoothing factor (for soft start)
float smoothingFactor = 0.1;

float initialSetpoint = 200.0; // Example initial setpoint
PIDController motorPID(kp, ki, kd, smoothingFactor, initialSetpoint);

// Time management
unsigned long lastTime = millis();

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  motorPID.setSetpoint(initialSetpoint);
}

void loop() {
  // Time step (dt)
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // Convert to seconds
  lastTime = now;

  // Read current motor speed (e.g., from sensor)
  float currentSpeed = analogRead(speedSensorPin); // Simulated speed sensor
  currentSpeed = map(currentSpeed, 0, 1023, 0, 255); // Map sensor value to speed

  // Calculate PID output
  float controlSignal = motorPID.compute(currentSpeed, dt);

  // Constrain the control signal between 0 and 255 (PWM range)
  controlSignal = constrain(controlSignal, 0, 255);

  // Set motor speed using PWM
  analogWrite(motorPin, controlSignal);

  // Debugging
  Serial.print("Current Speed: ");
  Serial.print(currentSpeed);
  Serial.print(" | Control Signal: ");
  Serial.println(controlSignal);

  delay(100); // Loop delay
}


