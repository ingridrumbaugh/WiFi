// Code for all components on the robot 
// Ingrid Rumbaugh 
// 3/7/2018 

#include <SPI.h>
#include <RH_NRF24.h>
#include <Servo.h>
#include <Stepper.h> 

// NRF PINS
#define CE   8
#define CSN  53
#define SCK  52
#define MOSI 51
#define MISO 50

// Instance of the radio driver
RH_NRF24 nrf24;

// VALUES FOR GAS SENSOR 
#define gas_sensor A0
int init_val; 
double sensor_vals[300];
double sensor_avg; // steady state average value 
int gaslevels[5]; 

// DRIVE MOTOR PINS
#define pwm1 30
#define pwm2 4

#define motor1Pin1 7 // INA
#define motor1Pin2 9 // INB 

#define motor2Pin1 5
#define motor2Pin2 6

// DRIVE MOTOR ENCODERS
#define enc1A 18
#define enc1B 19
#define enc2A 20
#define enc2B 21

// VALUES FOR DRIVE MOTOR ENCODERS 
int val1;
int enc1Pos = 0; 
int enc1ALast = LOW;
int n1 = LOW; 

int val2; 
int enc2Pos = 0; 
int enc2ALast = LOW; 
int n2 = LOW; 

// VALUES FOR ARM MOTOR ENCODER 
int val3; 
int enc3Pos = 0; 
int enc3ALast = LOW; 
int n3 = LOW; 

// ARM MOTOR PINS
#define arm_pwm1 42 // 51
#define arm_motor1_pin1 43 // 52
#define arm_motor1_pin2 44 // 53

// ARM ENCODERS 
#define enc3A 2
#define enc3B 3

// ARM SERVOS
#define servo1_pin 46

Servo servo1;
int servo1_pos = 0;

// ARM STEPPERS 
const int stepsPerRevA = (1/0.9)*360;
const int stepsPerRevW = (1/1.8)*360; 

Stepper armStepper(stepsPerRevA, 22, 23, 24, 25); 
Stepper wristStepper(stepsPerRevW, 26, 27, 28, 29); 

void setup() {
  pinMode(pwm1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(pwm2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(enc1A, INPUT);
  pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT);
  pinMode(enc2B, INPUT);

  pinMode(enc3A, INPUT);
  pinMode(enc3B, INPUT); 

  init_val = 300; 
  sensor_avg = 0; 
  
  armStepper.setSpeed(300); 
  wristStepper.setSpeed(300); 
  
  Serial.begin(9600);

  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
}

void loop() {

  read_motor_encoders();
  read_arm_encoders(); 
  read_gas_sensor(); 
  wristcnterclockwise(0); 
  delay(1000);
  wristclockwise(0); 
  delay(1000);
  
}

/**
 * Example NRF Server code - to be called in loop 
 */
void nrf_server() {

  if (nrf24.available()) {
    // Should be a message for us now

    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (nrf24.recv(buf, &len)) {
      // NRF24::printBuffer("request: ", buf, len);
      Serial.print("got request: ");
      Serial.println((char*)buf);

      // Send a reply
      uint8_t data[] = "And hello back to you";
      nrf24.send(data, sizeof(data));
      nrf24.waitPacketSent();
      Serial.println("Sent a reply");
    }
    else {
      Serial.println("recv failed");
    }
  }

}

/**
 * Example NRF Client code - to be called in loop 
 */
void nrf_client() {
  Serial.println("Sending to nrf24_server");

  // Send a message to nrf24_server
  uint8_t data[] = "Hello World!";
  nrf24.send(data, sizeof(data));

  nrf24.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (nrf24.waitAvailableTimeout(500)) {
    // Should be a reply message for us now

    if (nrf24.recv(buf, &len)) {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
    }
    else {
      Serial.println("recv failed");
    }
  }
  else {
    Serial.println("No reply, is nrf24_server running?");
  }

  delay(400);
}


void read_motor_encoders() {
  n1 = digitalRead(enc1A);
  n2 = digitalRead(enc2A); 

  // calculate encoder 1 
  if ((enc1ALast == LOW) && (n1 == HIGH)) {
    if (digitalRead(enc1B) == LOW) {
      enc1Pos --; 
    } else {
      enc1Pos ++; 
    }
  }

  // calculate encoder 2
  if ((enc2ALast == LOW) && (n2 == HIGH)) {
    if (digitalRead(enc2B) == LOW) {
      enc2Pos --; 
    } else {
      enc2Pos ++; 
    }
  }
  
}

void read_arm_encoders() {
  n3 = digitalRead(enc3A); 

  // calculate encoder 3
  if ((enc3ALast == LOW) && (n3 == HIGH)) {
    if (digitalRead(enc3B) == LOW) {
      enc3Pos --;
    } else {
      enc3Pos ++; 
    }
  }
  
}

void read_gas_sensor() {
  if (init_val > 0) {
    sensor_vals[300-init_val] = analogRead(A0); 
    init_val --; 
  }
  else if (init_val == 0) {
    double sum = 0; 
    for (int i = 0; i < 300; i ++) {
      sum += sensor_vals[i]; 
    }
    sensor_avg = sum / 300; 
    init_val = -1; 
  }
  if (init_val < 0) {
    double val = analogRead(A0)/sensor_avg; 
    if (val > 2) 
      writeLEDs(4, 255); 
    else 
      writeLEDs(4, 0); 
  }
  
}

void writeLEDs(int index, int analogval) {
  // Send analog val of red led to the MID 
  
  // send LED[index] = analogval 
}

void arm_servo_fwd() {
  for (int j = 0; j < 180; j ++) {
    servo1.write(j); 
  }
}

void arm_servo_bwd() {
  for (int k = 180; k < 0; k --) {
    servo1.write(k); 
  }
}

/**
 * Stop all motors 
 */
void fullStop() {
  motor1Pause();
  motor2Pause();
  armMotor1Pause();
}

/**
 * Drive motor 1 Forward 
 * Given PWM 0-255
 */
void motor1FWD(int pwm) {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(pwm1, pwm);
}

/**
 * Drive motor 1 backward 
 * Given PWM 0-255
 */
void motor1BWD(int pwm) {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(pwm1, pwm);
}

/**
 * Stop motor 1
 */
void motor1Pause() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(pwm1, 0);
}

/**
 * Drive motor 2 forward
 * Given PWM 0-255
 */
void motor2FWD(int pwm) {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(pwm2, pwm);
}

/**
 * Drive motor 2 backward
 * Given PWM 0-255
 */
void motor2BWD(int pwm) {
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(pwm2, pwm);
}

/**
 * Stop motor 2
 */
void motor2Pause() {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(pwm2, 0);
}

void wristclockwise(int timedelay) {
  Serial.println("Wrist clockwise");
  wristStepper.step(stepsPerRevW); 
  delay(timedelay); 
}

void wristcnterclockwise(int timedelay) {
  Serial.println("Wrist cnterclockwise");
  wristStepper.step(-stepsPerRevW); 
  delay(timedelay); 
}

void armclockwise(int timedelay) {
  Serial.println("clockwise");
  armStepper.step(stepsPerRevA);
  delay(timedelay); 
}

void armcnterclockwise(int timedelay) {
  Serial.println("counterclockwise");
  armStepper.step(-stepsPerRevA); 
  delay(timedelay); 
}

/**
 * Methods for the arm motor 
 */
void armMotor1FWD(int pwm) {
  digitalWrite(arm_motor1_pin1, HIGH);
  digitalWrite(arm_motor1_pin2, LOW);
  analogWrite(arm_pwm1, pwm);
}

void armMotor1BWD(int pwm) {
  digitalWrite(arm_motor1_pin1, LOW); 
  digitalWrite(arm_motor1_pin2, HIGH); 
  analogWrite(arm_pwm1, pwm); 
}

void armMotor1Pause() {
  digitalWrite(arm_motor1_pin1, HIGH); 
  digitalWrite(arm_motor1_pin2, HIGH); 
}



