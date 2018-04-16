// Code for all components on the robot
// Ingrid Rumbaugh
// 3/7/2018

#include <SPI.h>
#include <RH_NRF24.h>
#include <Servo.h>
#include <Stepper.h>
#include <avr/eeprom.h>

// NRF PINS
#define CE   8
#define CSN  53
#define SCK  52
#define MOSI 51
#define MISO 50

// Instance of the radio driver
RH_NRF24 nrf24;

// VALUES FOR GAS SENSOR
#define gasPin A0
int analogSensor;

// button values for MID Transmissions
int joystate1 = 0;
int joystate2 = 0;
int bstate[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// DRIVE MOTOR PINS
#define pwm1 40
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
bool arm_moving = false;
bool allow_rotate_cw = false;
bool allow_rotate_ccw = false;
int cw_max, ccw_max;

// ARM SERVOS
#define servo1_pin 46

Servo servo1;
int servo1_pos = 0;

// ARM STEPPERS
const int stepsPerRevA = (1 / 0.9) * 360;
const int stepsPerRevW = (1 / 1.8) * 360;

Stepper armStepper(stepsPerRevA, 22, 23, 24, 25);
Stepper wristStepper(stepsPerRevW, 26, 27, 28, 29);

// LIMIT SWITCHES
#define limit_up   48
#define limit_down 49
bool allow_up = false;
bool allow_down = false;

struct settings_t {
  int arm_enc_val;
} settings;

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

  pinMode(limit_up,   INPUT);
  pinMode(limit_down, INPUT);

  armStepper.setSpeed(50);
  wristStepper.setSpeed(50);
  servo1.attach(servo1_pin);

  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));
  enc3Pos = settings.arm_enc_val;

  Serial.begin(9600);
  Serial.println("Saved Encoder Value: " + (int)settings.arm_enc_val);

  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
}

void loop() {
  receiveMessage();
  delay(100);
  if (arm_moving == true) {
    read_arm_encoders();
  }
  //  read_motor_encoders();
  //  read_gas_sensor();
  //  armcnterclockwise(0);
  //  armclockwise(0);
  //  delay(500);
  //  delay(1000);
  //  arm_servo_fwd();
  //  delay(1000);
  //  arm_servo_bwd();
  //  delay(1000);
  //  armMotor1FWD(100);
  //  delay(500);
  //  armMotor1BWD(100);
  //  delay(500);
  //  wristcnterclockwise(500);
  //  wristclockwise(500);
  //  delay(500);
  sendMessage();
}

/**
   Example NRF Server code - to be called in loop
*/
void receiveMessage() {
  Serial.println("Receiving from MID");

  //Define Input (Currently 3 bytes)
  byte buf[3] = {};
  uint8_t len = sizeof(buf);

  //Wait for message
  if (nrf24.recv((byte*)buf, &len)) {
    Serial.print("Got reply.");
  }
  else {
    Serial.println("Receive failed.");
    return;
  }

  joystate1 = buf[0];
  joystate2 = buf[1];

  Serial.print("Joystick 1: ");
  Serial.println(joystate1);
  Serial.print("Joystick 2: ");
  Serial.println(joystate2);

  byte bString = buf[2];

  Serial.print("Buttonstring: ");
  Serial.println(bString);
  Serial.print("Buttonstate Parsing: ");

  for (int j = 0; j < 8; j++) {
    bstate[7 - j] = (bString >> (7 - j)) % 2;
    Serial.print(bstate[7 - j]);
  }
  Serial.println();
}

/**
   Example NRF Client code - to be called in loop
*/
void sendMessage() {
  Serial.println("Sending to MID");

  byte gas[1];
  uint8_t temp = 101;

  //MESS[0] SHOULD BE THE GAS SENSOR DATA
  gas[0] = (byte)temp;
  nrf24.send(gas, sizeof(gas));
  nrf24.waitPacketSent();

  return;
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
      settings.arm_enc_val --;
      // DOUBLE CHECK THIS:
      if (settings.arm_enc_val < cw_max) {
        allow_rotate_cw = false;
      }
    } else {
      enc3Pos ++;
      settings.arm_enc_val ++;
      if (settings.arm_enc_val > ccw_max) {
        allow_rotate_ccw = false;
      }
    }
  }
  Serial.println();
  Serial.print("Encoder: ");
  Serial.print(settings.arm_enc_val);
  Serial.println();
  Serial.println("Writing EEPROM");
  write_eeprom();
}

void write_eeprom() {
  eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));
}

/**
   Check button values and run shit
*/
void run_robot() {

}

void read_gas_sensor() {
  analogSensor = analogRead(gasPin);
}

void arm_servo_fwd() {
  int pos = 0;
  for (pos = 0; pos <= 180; pos += 1) {
    // in steps of 1 degree
    servo1.write(pos);
    delay(15);
  }
}

void arm_servo_bwd() {
  int pos = 0;
  for (pos = 90; pos >= 0; pos -= 1) {
    servo1.write(pos);
    delay(15);
  }
}

/**
   Stop all motors
*/
void fullStop() {
  motor1Pause();
  motor2Pause();
  armMotor1Pause();
}

/**
   Drive motor 1 Forward
   Given PWM 0-255
*/
void motor1FWD(int pwm) {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(pwm1, pwm);
}

/**
   Drive motor 1 backward
   Given PWM 0-255
*/
void motor1BWD(int pwm) {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(pwm1, pwm);
}

/**
   Stop motor 1
*/
void motor1Pause() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(pwm1, 0);
}

/**
   Drive motor 2 forward
   Given PWM 0-255
*/
void motor2FWD(int pwm) {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(pwm2, pwm);
}

/**
   Drive motor 2 backward
   Given PWM 0-255
*/
void motor2BWD(int pwm) {
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(pwm2, pwm);
}

/**
   Stop motor 2
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
  // CHECK THIS:
  check_limit_up(); 
  if (allow_up == true) {
    Serial.println("clockwise");
    armStepper.step(stepsPerRevA);
    delay(timedelay);
  } else {
    return;
  }
}

void armcnterclockwise(int timedelay) {
  // CHECK THIS:
  check_limit_down(); 
  if (allow_down == true) {
    Serial.println("counterclockwise");
    armStepper.step(-stepsPerRevA);
    delay(timedelay);
  } else {
    return;
  }
}

/**
   Methods for the arm motor
*/
void armMotor1FWD(int pwm) {
  // CHECK THIS
  if (allow_rotate_cw == true) {
    arm_moving = true;
    digitalWrite(arm_motor1_pin1, HIGH);
    digitalWrite(arm_motor1_pin2, LOW);
    analogWrite(arm_pwm1, pwm);
  } else {
    armMotor1Pause();
    return;
  }

}

void armMotor1BWD(int pwm) {
  if (allow_rotate_ccw == true) {
    arm_moving = true;
    digitalWrite(arm_motor1_pin1, LOW);
    digitalWrite(arm_motor1_pin2, HIGH);
    analogWrite(arm_pwm1, pwm);
  } else {
    armMotor1Pause();
    return;
  }
}

void armMotor1Pause() {
  arm_moving = false;
  digitalWrite(arm_motor1_pin1, HIGH);
  digitalWrite(arm_motor1_pin2, HIGH);
}

bool check_limit_up() {
  if (digitalRead(limit_up) == LOW) {
    allow_up = false;
  } else {
    allow_up = true;
  }
  return allow_up;
}

bool check_limit_down() {
  if (digitalRead(limit_down) == LOW) {
    allow_down = false;
  } else {
    allow_down = true;
  }
  return allow_down;
}




