// THIS CODE IS FOR AN UNO
#include <SPI.h>
#include <RH_NRF24.h>
//#include <Servo.h>

// NRF PINS
#define CE   8
#define CSN  10
#define SCK  13
#define MOSI 11
#define MISO 12

// Singleton instance of the radio driver
RH_NRF24 nrf24;

// DRIVE MOTOR PINS
#define pwm1 3
#define pwm2 4

#define motor1Pin1 7 // INA
#define motor1Pin2 9 // INB --> USED TO BE PIN 8!!

#define motor2Pin1 5
#define motor2Pin2 6

// DRIVE MOTOR ENCODERS
#define enc1A 18
#define enc1B 19
#define enc2A 20
#define enc2B 21

volatile unsigned int countGlobal1 = 0;
volatile unsigned int countGlobal2 = 0; 

// 64 CPR -> 100:1 GR -> 4X Encoder
double radConvert = 360.0 / 1600.0;
double theta1 = 0;
double theta2 = 0; 

long t = 0;
long tstart = 0;
long treset = 0;
long tperp = 0;

// Moving fwd
boolean forward = false;
// Moving bwd
boolean backward = false;
// Wait before moving fwd
boolean waitF = true;
// Wait before moving bwd
boolean waitB = false;

// ARM MOTOR PINS
#define arm_pwm1 51
#define arm_pwm2 47

#define arm_motor1_pin1 52
#define arm_motor1_pin2 53

#define arm_motor2_pin1 49
#define arm_motor2_pin2 48

// ARM SERVOS
#define servo1_pin 46
//Servo servo1;
int servo1_pos = 0;

void setup() {
  pinMode(pwm1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(pwm2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  //servo1.attach(servo1_pin);
  //attach the interrupts for encoder channels to interrupts 0 and 1
  //(pins 2 and 3).
  attachInterrupt(enc1A, tachRead1, CHANGE);
  attachInterrupt(enc2A, tachRead2, CHANGE);

  pinMode(enc1A, INPUT);
  pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT);
  pinMode(enc2B, INPUT);
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

  tstart = millis();
  treset = tstart;

}

void loop() {
  // Zero values
  t = millis() - treset;
  tperp = millis() - tstart;

  motor1FWD(100);
  motor2FWD(100);
//  delay(500); 
//  motor1Pause();
//  motor2Pause();
//  delay(500);
//  motor1BWD(100);
//  motor2BWD(100); 
//  delay(500); 
//  motor1Pause();
//  motor2Pause();
//  delay(500); 

  noInterrupts();
  long countLocal1 = countGlobal1;
  long countLocal2 = countGlobal2; 
  interrupts();
  theta1 = countLocal1*radConvert; 
  theta2 = countLocal2*radConvert; 
  
  // Set up next values
//  Serial.print("TPERP");
//  Serial.print("\t");
//  Serial.print("ENC1A");
//  Serial.print("\t");
//  Serial.print("ENC1B");
//  Serial.print("\t");
//  Serial.print("ENC2A");
//  Serial.print("\t");
//  Serial.print("ENC2B");
//  Serial.println("  "); 
//  Serial.println("  "); 
//  Serial.print(tperp / 1000.0);
//  Serial.print("\t");
//  Serial.print(digitalRead(enc1A));
//  Serial.print("\t");
//  Serial.print(digitalRead(enc1B));
//  Serial.print("\t");
//  Serial.print(digitalRead(enc2A));
//  Serial.print("\t");
//  Serial.print(digitalRead(enc2B));
  Serial.println("    ");
  Serial.print("Theta1:    "); 
  Serial.print(theta1);
  Serial.println("    "); 
  Serial.print("Theta 2:   ");
  Serial.print(theta2); 
  Serial.println("    "); 
}


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

//
//void servoTest() {
//  for (servo1_pos = 0; servo1_pos <= 180; servo1_pos += 1) { // goes from 0 degrees to 180 degrees
//    servo1.write(servo1_pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//
//  for (servo1_pos = 180; servo1_pos >= 0; servo1_pos -= 1) { // goes from 180 degrees to 0 degrees
//    servo1.write(servo1_pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//}

void fullStop() {
  motor1Pause();
  motor2Pause();
  armMotor1Pause();
  armMotor2Pause();
}

void tachRead1() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) {
    countGlobal1++; // increase count by one
  }
  else {
    countGlobal1--; // decrease count by one
  }
}

void tachRead2() {
  
   if (digitalRead(enc2B) == digitalRead(enc2B)) {
    countGlobal2++;
  }
  else {
    countGlobal2--;
  }
}

void motor1FWD(int pwm) {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(pwm1, pwm);
}

void motor1BWD(int pwm) {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(pwm1, pwm);
}

void motor1Pause() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(pwm1, 0);
}

void motor2FWD(int pwm) {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(pwm2, pwm);
}

void motor2BWD(int pwm) {
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(pwm2, pwm);
}

void motor2Pause() {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(pwm2, 0);
}

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













