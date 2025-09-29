// === Pin Configuration ===
const int EN = 5;     // PWM pin to L298N ENA
const int IN1 = 6;    // Direction pin
const int IN2 = 7;    // Direction pin
const int encoderPinA = 2; // Encoder Channel A (must support interrupt)

// === PID Constants ===
float Kp = 0.27;
float Ki = 0.08;
float Kd = 0.0085;

// === Motor Control Variables ===
volatile long encoderCount = 0;
long prevEncoderCount = 0;
float motorSpeed = 0;
float setSpeed = 200.0;  // Target speed (counts/sec)
float error, prevError = 0, integral = 0;
int pwm = 0;

unsigned long lastTime = 0;
unsigned long interval = 250; // 100ms interval

// === Interrupt for Encoder Count ===
void encoderISR() {
  encoderCount++;
}

void setup() {
  Serial.begin(9600);

  pinMode(EN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(encoderPinA, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);

  // Set initial motor direction
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;

    // Calculate speed (counts per second)
    long currentCount = encoderCount;
    motorSpeed = (currentCount - prevEncoderCount) * (1000.0 / interval);
    prevEncoderCount = currentCount;

    // === PID Control ===
    error = setSpeed - motorSpeed;
    integral += error * (interval / 1000.0);
    float derivative = (error - prevError) / (interval / 1000.0);
    float output = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;

    // PWM output
    pwm = (int)output;

    // Avoid motor stalling at low PWM values
    if (pwm > 0 && pwm < 60) pwm = 60;
    
    // Clamp to 0–255
    pwm = constrain(pwm, 0, 255);
    analogWrite(EN, pwm);

    // === Debug Output ===
    Serial.print("Target: ");
    Serial.print(setSpeed);
    Serial.print(" | Speed: ");
    Serial.print(motorSpeed);
    Serial.print(" | PWM: ");
    Serial.print(pwm);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | PID Output: ");
    Serial.println(output);

  }
}
