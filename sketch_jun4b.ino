#include "Wire.h"                  
#include "HMC5883L.h"                                            
#include <SoftwareSerial.h>                                       // Software Serial for Serial Communications - not used
#include <TinyGPS++.h>    

// GPS Variables & Setup

int GPS_Course;                                                    // variable to hold the gps's determined course to destination
int Number_of_SATS;                                                // variable to hold the number of satellites acquired
TinyGPSPlus gps;                                                   // gps = instance of TinyGPS 



//******************************************************************************************************
// Compass Variables & Setup

HMC5883L compass;                                                  // HMC5883L compass(HMC5883L)
int16_t mx, my, mz;                                                // variables to store x,y,z axis from compass (HMC5883L)
int desired_heading;                                               // initialize variable - stores value for the new desired heading
int compass_heading;                                               // initialize variable - stores value calculated from compass readings
int compass_dev = 5;                                               // the amount of deviation that is allowed in the compass heading - Adjust as Needed
                                                                   // setting this variable too low will cause the robot to continuously pivot left and right
                                                                   // setting this variable too high will cause the robot to veer off course

int Heading_A;                                                     // variable to store compass heading
int Heading_B;                                                     // variable to store compass heading in Opposite direction
int pass = 0;                                                      // variable to store which pass the robot is on

bool turnLeftFlag = false;
bool turnRightFlag = false;
int initialHeading = 0;
int targetHeading = 0;


//******************************************************************************************************
// Ping Sensor for Collision Avoidance

boolean pingOn = false;                                            // Turn Collision detection On or Off

int trigPin = 29;                                                   
int echoPin = 30;                                                  
long duration, distance;
int Ping_distance;


// GPS Locations

unsigned long Distance_To_Home;                                    

int ac =0;                                                         
int wpCount = 0;                                                   
double Home_LATarray[50];                                          
double Home_LONarray[50];                                          


int increment = 0;


// === Pin Configuration ===
// Motor A
const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int encoderA = 2;

// Motor B
const int ENB = 9;
const int IN3 = 11;
const int IN4 = 10;
const int encoderB = 3;

// === PID Constants ===
float Kp = 0.27;
float Ki = 0.08;
float Kd = 0.0085;

// === Motor A Variables ===
volatile long encoderCountA = 0;
long prevCountA = 0;
float speedA = 0;
float errorA, prevErrorA = 0, integralA = 0;
int pwmA = 0;

// === Motor B Variables ===
volatile long encoderCountB = 0;
long prevCountB = 0;
float speedB = 0;
float errorB, prevErrorB = 0, integralB = 0;
int pwmB = 0;

// === Common Control ===
unsigned long lastTime = 0;
unsigned long interval = 250; // ms
float setSpeed = 200.0;
extern char direction = 'S'; // Start stopped
bool sendingflag =false;
char incoming;
// === ISRs ===
void encoderA_ISR() { encoderCountA++; }
void encoderB_ISR() { encoderCountB++; }
// Encoder and wheel settings
const int pulsesPerMotorRev = 7;
const float gearRatio = 46.8;
const float ticksPerWheelRev = pulsesPerMotorRev * gearRatio;  // ≈ 328 ticks
const float wheelDiameter = 0.065;  // in meters
const float wheelCircumference = PI * wheelDiameter;  // ≈ 0.2042 m
const float wheelBase = 0.145;  // in meters (14.5 cm)

float x = 0, y = 0, theta = 0;  // Robot pose in meters and radians

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); 
  Serial1.println("VINI CAR");
  // Motor A setup
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(encoderA, INPUT);

  // Motor B setup
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(encoderB, INPUT);
  pinMode(trigPin, OUTPUT);                                        // Ping Sensor
  pinMode(echoPin, INPUT);                                         // Ping Sensor

  // Compass
  Wire.begin();                                                    // Join I2C bus used for the HMC5883L compass
  compass.begin();                                                 // initialize the compass (HMC5883L)
  compass.setRange(HMC5883L_RANGE_1_3GA);                          // Set measurement range  
  compass.setMeasurementMode(HMC5883L_CONTINOUS);                  // Set measurement mode  
  compass.setDataRate(HMC5883L_DATARATE_30HZ);                     // Set data rate  
  compass.setSamples(HMC5883L_SAMPLES_8);                          // Set number of samples averaged  
  compass.setOffset(283,75,0);   

  attachInterrupt(digitalPinToInterrupt(encoderA), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderB_ISR, RISING);
}

void loop() {
  static long prevLeftTicks = 0;
  static long prevRightTicks = 0;
  // === Serial Input ===
  if (Serial1.available() > 0) {
    char cmd = Serial1.read();
    cmd = toupper(cmd); // Convert to uppercase
    incoming = cmd;
    if (incoming == 'F' || incoming == 'B' || incoming == 'L' || incoming == 'R' || incoming == 'S' || incoming == 'I' || incoming == 'W' || incoming == 'Y' || incoming == 'D' || incoming == 'C' || incoming == 'G' || incoming == 'H' || incoming == 'T' ) {
      direction = incoming;
      Serial.print("Received direction: "); Serial.println(direction);
      sendingflag = true;
    }
  if (incoming == 'Q') {  // Left 90
    setHeading();
    initialHeading = compass_heading;
    targetHeading = (initialHeading - 90 + 360) % 360;
    direction = incoming;
    sendingflag = true;
    turnLeftFlag = true;
    Serial1.println("Turning Left 90");
    Serial.println("Turning Left 90");
  } 
  else if (incoming == 'E') {  // Right 90
    setHeading();
    initialHeading = compass_heading;
    targetHeading = (initialHeading + 90) % 360;
    direction = incoming;
    turnRightFlag = true;
    sendingflag = true;
    Serial1.println("Turning Right 90");
  }
   if(incoming == 'S'){
    turnRightFlag = false;
    turnLeftFlag = false;
   }
}

  

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;
  long dLeft = encoderCountA - prevLeftTicks;
  long dRight = encoderCountB - prevRightTicks;

  prevLeftTicks = encoderCountA;
  prevRightTicks = encoderCountB;

    // === Direction Setup ===
    setDirection(direction);
    Ping();
    // === Speed Calculation ===
    long countA = encoderCountA;
    speedA = (countA - prevCountA) * (1000.0 / interval);
    prevCountA = countA;

    long countB = encoderCountB;
    speedB = (countB - prevCountB) * (1000.0 / interval);
    prevCountB = countB;

    // === Skip PID if stopped ===
    if (setSpeed == 0) {
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      return;
    }

    // === PID Motor A ===
    errorA = setSpeed - speedA;
    integralA += errorA * (interval / 1000.0);
    float derivativeA = (errorA - prevErrorA) / (interval / 1000.0);
    float outputA = Kp * errorA + Ki * integralA + Kd * derivativeA;
    prevErrorA = errorA;
    pwmA = (int)outputA;
    if (pwmA > 0 && pwmA < 60) pwmA = 60;
    pwmA = constrain(pwmA, 0, 255);
    analogWrite(ENA, pwmA);

    // === PID Motor B ===
    errorB = setSpeed - speedB;
    integralB += errorB * (interval / 1000.0);
    float derivativeB = (errorB - prevErrorB) / (interval / 1000.0);
    float outputB = Kp * errorB + Ki * integralB + Kd * derivativeB;
    prevErrorB = errorB;
    pwmB = (int)outputB;
    if (pwmB > 0 && pwmB < 60) pwmB = 60;
    pwmB = constrain(pwmB, 0, 255);
    analogWrite(ENB, pwmB);
  float distLeft = (dLeft * wheelCircumference) / ticksPerWheelRev;
  float distRight = (dRight * wheelCircumference) / ticksPerWheelRev;

  float dist = (distLeft + distRight) / 2.0;
  float deltaTheta = (distRight - distLeft) / wheelBase;

  // Update pose
  x += dist * cos(theta + deltaTheta / 2.0);
  y += dist * sin(theta + deltaTheta / 2.0);
  theta += deltaTheta;

  // Normalize angle between -π and π
  if (theta > PI) theta -= 2 * PI;
  if (theta < -PI) theta += 2 * PI;

  Serial.print("x: "); Serial.print(x, 3);
  Serial.print(" m, y: "); Serial.print(y, 3);
  Serial.print(" m, θ: "); Serial.print(theta * 180.0 / PI, 2);
  Serial.println("°");
    // === Debug Info ===
    Serial.print("Dir: "); Serial.print(direction);
    Serial.print(" | A_Speed: "); Serial.print(speedA);
    Serial.print(" | A_PWM: "); Serial.print(pwmA);
    Serial.print(" | B_Speed: "); Serial.print(speedB);
    Serial.print(" | B_PWM: "); Serial.println(pwmB);
  }
}

// === Set Direction Logic ===
void setDirection(char dir) {
  if(sendingflag){
  switch (dir) {
    case 'F': // Forward
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      setSpeed = 200;
      Ping();
      break;
    case 'B': // Backward
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      setSpeed = 200;
      break;
    case 'L': // Turn left
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  // Left reverse
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Right forward
      setSpeed = 100;
      break;
    case 'R': // Turn right
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Left forward
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  // Right reverse
      setSpeed = 100;
      break;
    case 'S': // Stop
      stopMotors();
      break;
    case 'I':
      gpsInfo();
      break;
    case 'Y':
      setWaypoint();
      break;
    case 'W':
      goWaypoint();
      break;
    case 'D':
      ac = 0;
      Serial1.print("Waypoints Complete");
      break;
    case 'C':
      clearWaypoints();
      break;
    case 'H':
      setHeading();
    break;
      

  }
  }
  if (turnLeftFlag || turnRightFlag) {
  setHeading();  // update compass_heading

  //  Check for incoming stop command
  if (Serial1.available()) {
    char x = Serial1.read();
    if (x == 'S') {
      stopMotors();
      turnLeftFlag = false;
      turnRightFlag = false;
      Serial1.println("Stopped by user");
    return;   
    }
  }

  int currentHeading = compass_heading;
  int error = (targetHeading - currentHeading + 360) % 360;

  //  Handle Left Turn
  if (turnLeftFlag) {
    if (error < 5 || error > 355) {  // Close enough to target
      stopMotors();
      turnLeftFlag = false;
      Serial1.println("Left Turn Complete");
    } else {
      // Spin left
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  // Left reverse
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Right forward
      setSpeed = 100;
    }
  }

  //  Handle Right Turn
  else if (turnRightFlag) {
    if (error < 10 || error > 350) {
      stopMotors();
      turnRightFlag = false;
      Serial1.println("Right Turn Complete");
    } else {
      // Spin right
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Left forward
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  // Right reverse
      setSpeed = 100;
    }
  }
}


}

// Utility Function to stop all motors
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  setSpeed = 0;
}
