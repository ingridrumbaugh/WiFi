// Code for all components on the robot 
// Ingrid Rumbaugh 
// 3/7/2018 

#include <SPI.h>
#include <RH_NRF24.h>
#include <Servo.h>
#include <Stepper.h> 

// NRF PINS
#define CE   8
#define CSN  10
#define SCK  13
#define MOSI 11
#define MISO 12

// Instance of the radio driver
RH_NRF24 nrf24;

// DRIVE MOTOR PINS
#define pwm1 3
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

// ARM MOTOR PINS
#define arm_pwm1 51
#define arm_pwm2 47 // NOT 41!

#define arm_motor1_pin1 52
#define arm_motor1_pin2 53

#define arm_motor2_pin1 49
#define arm_motor2_pin2 48

// ARM SERVOS
#define servo1_pin 46
Servo servo1;
int servo1_pos = 0;

// ARM STEPPERS 
// start @ 22 
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

  armStepper.setSpeed(100); 
  wristStepper.setSpeed(100); 
  
  Serial.begin(9600);

  //if the voltage on pin 2 changes, run the channelA subroutine
  //  while (!Serial)
  //    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
}

void loop() {
  // Run both motors forward 
  motor1FWD(100);
  motor2FWD(100);
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


/**
 * Servo Demo - Run servo forwards and backwards
 */
void servoTest() {
  for (servo1_pos = 0; servo1_pos <= 180; servo1_pos += 1) { 
    // goes from 0 degrees to 180 degrees
    servo1.write(servo1_pos);              
    // tell servo to go to position in variable 'pos'
    delay(15);                       
    // waits 15ms for the servo to reach the position
  }

  for (servo1_pos = 180; servo1_pos >= 0; servo1_pos -= 1) { 
    // goes from 180 degrees to 0 degrees
    servo1.write(servo1_pos);              
    // tell servo to go to position in variable 'pos'
    delay(15);                       
    // waits 15ms for the servo to reach the position
  }
}

/**
 * Stop all motors 
 */
void fullStop() {
  motor1Pause();
  motor2Pause();
  armMotor1Pause();
  armMotor2Pause();
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
 * Empty methods for the arm motor 
 */
void armMotor1FWD(int pwm) {

}

void armMotor1BWD(int pwm) {

}

void armMotor1Pause() {

}

void armMotor2FWD(int pwm) {

}

void armMotor2BWD(int pwm) {

}

void armMotor2Pause() {

}


