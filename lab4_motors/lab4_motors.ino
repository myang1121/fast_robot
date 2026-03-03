// Task 7 - running the car on the ground
// This test program tests the car's ability to move forward, backward, turn left, and right

// Motor 1 (right, front view) pins
#define MOTOR1_IN_PLUS 12
#define MOTOR1_IN_MINUS A15


// Motor 2 (left) pins


int pwm_speed = 150; // PWM value 0-255

void setup() {
  Serial.begin(115200);
  // Set as output (from Artemis, input to motor controller)
  pinMode(MOTOR1_IN_PLUS, OUTPUT);
  pinMode(MOTOR1_IN_MINUS, OUTPUT);
}

void loop() {
  // move forward for 5 seconds
  Serial.println("Move forward");
  forward(pwm_speed);
  delay(5000);

  // stop for 5 second
  Serial.println("Stop!");
  stop();
  delay(5000); 

  // move backward for 5 seconds
  Serial.println("Move backward");
  backward(pwm_speed);
  delay(5000);

  // stop for 5 second
  Serial.println("Stop!");
  stop();
  delay(5000); 

}

// movement functions
void forward(int speed) {
  analogWrite(MOTOR1_IN_PLUS, speed);
  analogWrite(MOTOR1_IN_MINUS, 0);
}

void backward(int speed) {
  analogWrite(MOTOR1_IN_PLUS, 0);
  analogWrite(MOTOR1_IN_MINUS, speed);
}

// movement functions
void stop() {
  analogWrite(MOTOR1_IN_PLUS, 0);
  analogWrite(MOTOR1_IN_MINUS, 0);
}