// Code for all components on the robot
// Ingrid Rumbaugh
// 3/7/2018

#include <SPI.h>
#include <RH_NRF24.h>
#include <RHReliableDatagram.h>
#include <Servo.h>
#include <Stepper.h>
#include <avr/eeprom.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// NRF PINS
#define CE   8
#define CSN  53
#define SCK  52
#define MOSI 51
#define MISO 50

// Instance of the radio driver
RH_NRF24 nrf24(8, 53);

RHReliableDatagram manager(nrf24, SERVER_ADDRESS);

// VALUES FOR GAS SENSOR
#define gasPin A0
int analogSensor;

int failCounter = 0;

// button values for MID Transmissions
int joystate1 = 0;
int joystate2 = 0;
int bstate[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// DRIVE MOTOR PINS
#define pwm1 13
#define pwm2 4

#define motor1Pin1 7 // INA
#define motor1Pin2 9 // INB 

#define motor2Pin1 5 // INA 
#define motor2Pin2 6 // INB

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

// VALUES FOR WRIST MOTOR ENCODER
int val3;
int enc3Pos = 0;
int enc3ALast = LOW;
int n3 = LOW;

// VALUES FOR ARM MOTOR ENCODER
// NOTE: No pins for this encoder have been defined yet
int val4;
int enc4Pos = 0;
int enc4ALast = LOW;
int n4 = LOW;

// WRIST MOTOR PINS
#define wrist_pwm 10
#define wrist_motor_pin1 11
#define wrist_motor_pin2 12

// ARM MOTOR PINS
#define arm_pwm 43
#define arm_motor_pin1 45
#define arm_motor_pin2 47

// WRIST ENCODERS
#define enc3A 2
#define enc3B 3
bool wrist_moving = false;
bool allow_rotate_cw = false; // for the wrist
bool allow_rotate_ccw = false;
int cw_max, ccw_max;

// ARM SERVOS
#define servo1_pin 46
Servo servo1;
int servo1_pos = 0;
int pos = 0;

// CLAW STEPPER
const int stepsPerRevC = (1 / 1.8) * 360;
Stepper clawStepper(stepsPerRevC, 26, 27, 28, 29);

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

  pinMode(wrist_pwm, OUTPUT);
  pinMode(wrist_motor_pin1, OUTPUT);
  pinMode(wrist_motor_pin2, OUTPUT);

  pinMode(arm_pwm, OUTPUT);
  pinMode(arm_motor_pin1, OUTPUT);
  pinMode(arm_motor_pin2, OUTPUT);

  pinMode(enc1A, INPUT);
  pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT);
  pinMode(enc2B, INPUT);

  pinMode(enc3A, INPUT);
  pinMode(enc3B, INPUT);

  pinMode(limit_up,   INPUT);
  pinMode(limit_down, INPUT);

  // UNITS: ROTATIONS / MINUTE
  clawStepper.setSpeed(100);
  servo1.attach(servo1_pin);

  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));
  enc3Pos = settings.arm_enc_val;

  Serial.begin(115200);
  Serial.println("Saved Encoder Value: " + (int)settings.arm_enc_val);

  if (!manager.init())
    Serial.println("init failed");

  nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);

  manager.setRetries(2);
  manager.setTimeout(10);
}

void loop() {
  double a = millis();
  receiveMessage();
  double b = millis();

  Serial.print("Receive Took: ");
  Serial.println(b - a);
  Serial.println(" ");
  Serial.print("Joystate1: ");
  Serial.print(joystate1);
  Serial.println(" ");
  Serial.print("Joystate2: ");
  Serial.print(joystate2);
  Serial.println("  ");

  if (failCounter < 9 && joystate1 <= 114) {
    Serial.println("M1 Forward");
    motor1FWD((128 - joystate1)*2);

  } else if (failCounter < 9 && 142 <= joystate1) {
    Serial.println("M1 Backward");
    motor1BWD((joystate1 - 128)*2);

  } else {
    Serial.println("Pause");
    motor1Pause();
  }

  if (failCounter < 5 && joystate2 <= 114) {
    Serial.println("M2 Forward");
    motor2FWD((128 - joystate2));

  } else if (failCounter < 5 && 142 <= joystate2) {
    Serial.println("M2 Backward");
    motor2BWD((joystate2 - 128));

  } else {
    Serial.println("Pause");
    motor2Pause();
  }
  /**
     Button 4
  */
  if (bstate[3] == 1) {
    Serial.println("Opening Claw");
    open_claw(0);
  }

  /**
     Button 7
  */
  if (bstate[6] == 1) {
    Serial.println("Arm Servo FWD");
    arm_servo_fwd();
  }

  /**
     Button 3
  */
  if (bstate[2] == 1) {
    Serial.println("Wrist Clockwise");
    wrist_clockwise(150);
  }
  else if (failCounter >= 2 && bstate[2] == 0) {
    wristPause();
  }

  /**
     Button 2
  */
  if (bstate[1] == 1) {
    Serial.println("Closing Claw");
    close_claw(0);
  }

  /**
     Button 6
  */
  if (failCounter < 5 && bstate[5] == 1) {
    Serial.println("Arm Motor Up");
    armMotorUP(150);
  }
  else if (bstate[5] == 0) {
    armMotorPause();
  }

  /**
     Button 1
  */
  if (bstate[0] == 1) {
    Serial.println("Wrist Counterclockwise");
    wrist_cnterclockwise(150);
  }
  else if (failCounter >= 2 && bstate[5] == 0) {
    wristPause();
  }

  /**
     Button 5
  */
  if (bstate[4] == 1) 
  {
    Serial.println("Arm Servo BWD");
    arm_servo_bwd();
  }

  /**
     Button 8
  */
    
  if (failCounter < 5 && bstate[7] == 1) {
    Serial.println("Arm Motor Down");
    armMotorDOWN(150);
  }
  else if (bstate[7] == 0) {
    armMotorPause();
  }
  
  a = millis();
  sendMessage();
  b = millis();

  Serial.print("Send Took: ");
  Serial.println(b - a);
}

/**
  NRF Server code - to be called in loop
*/
void receiveMessage() {
  Serial.println("Receiving from MID");

  //Define Input (Currently 3 bytes)

  byte buf[3] = {};
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (manager.recvfromAckTimeout((byte*)buf, &len, 10, &from))
  {
    Serial.println("Got reply.");
    failCounter = 0;
  }
  else
  {
    Serial.println("No reply.");
    failCounter++;
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

  for (int j = 0; j < 8; j++)
  {
    bstate[7 - j] = (bString >> (7 - j)) % 2;
    Serial.print(bstate[7 - j]);
  }

  //delay(50);

  Serial.println();
}

/**
  NRF Client code - to be called in loop
*/
void sendMessage() {
  Serial.println("Sending to MID");

  byte gas[1];


  double a = millis();
  read_gas_sensor();
  double b = millis();

  Serial.print("Gas Sensor: ");
  Serial.println(b - a);

  uint8_t temp = analogSensor;

  //MESS[0] SHOULD BE THE GAS SENSOR DATA
  gas[0] = (byte)temp;

  //Send Message
  if (manager.sendtoWait(gas, sizeof(gas), CLIENT_ADDRESS)) {
    Serial.println("Message Sent.");
  }
  else
    Serial.println("sendtoWait failed.");
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

// wrist
void read_wrist_encoders() {
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

void read_gas_sensor() {
  analogSensor = analogRead(gasPin);
}

void arm_servo_fwd() {
  for (int i = pos; i <= 90; i += 1) {
    // in steps of 1 degree
    servo1.write(i);
    Serial.println(" ");
    Serial.print("ARM SERVO FWD:");
    Serial.print(i);
    Serial.println(" ");
    delay(10);
  }
  pos = 90;
  est_conn(); // re-establish connection 
}

void arm_servo_bwd() {
  for (int i = pos; i >= 0; i -= 1) {
    servo1.write(i);
    Serial.println(" ");
    Serial.print("ARM SERVO BWD:");
    Serial.print(i);
    Serial.println(" ");
    delay(10);
  }
  pos = 0;
  est_conn();// re-establish connection 
}

void est_conn() {
  if (!manager.init())
    Serial.println("init failed");

  nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);

  manager.setRetries(2);
  manager.setTimeout(10);
}

/**
   Stop all motors
*/
void fullStop() {
  motor1Pause();
  motor2Pause();
  wristPause();
  armMotorPause();
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
void motor2BWD(int pwm) {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(pwm2, pwm);
}

/**
   Drive motor 2 backward
   Given PWM 0-255
*/
void motor2FWD(int pwm) {
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

// used to be wristclockwise
void close_claw(int timedelay) {
  Serial.println("Wrist clockwise");
  clawStepper.step(stepsPerRevC);
  delay(timedelay);
}

void open_claw(int timedelay) {
  Serial.println("Wrist cnterclockwise");
  clawStepper.step(-stepsPerRevC);
  delay(timedelay);
}

/**
   Methods for the arm motor
*/
void wrist_clockwise(int pwm) {
  read_wrist_encoders();
  // CHECK THIS
  //  if (allow_rotate_cw == true) {
  wrist_moving = true;

  digitalWrite(wrist_motor_pin1, HIGH);
  digitalWrite(wrist_motor_pin2, LOW);
  analogWrite(wrist_pwm, pwm);
  //  } else {
  //    wristPause();
  //    return;
  //  }
}

void wrist_cnterclockwise(int pwm) {
  read_wrist_encoders();
  //  if (allow_rotate_ccw == true) {
  wrist_moving = true;
  digitalWrite(wrist_motor_pin1, LOW);
  digitalWrite(wrist_motor_pin2, HIGH);
  analogWrite(wrist_pwm, pwm);
  //  } else {
  //    wristPause();
  //    return;
  //  }
}

void wristPause() {
  read_wrist_encoders();
  wrist_moving = false;
  digitalWrite(wrist_motor_pin1, LOW);
  digitalWrite(wrist_motor_pin2, LOW);
  analogWrite(wrist_pwm, 0);
}

void armMotorUP(int pwm) {
  check_limit_up();
//  if (allow_up == true) {
    digitalWrite(arm_motor_pin1, HIGH);
    digitalWrite(arm_motor_pin2, LOW);
    analogWrite(arm_pwm, pwm);
//  } else {
//    armMotorPause();
//    return;
//  }
}

void armMotorDOWN(int pwm) {
  check_limit_down();
//  if (allow_down == true) {
    digitalWrite(arm_motor_pin1, LOW);
    digitalWrite(arm_motor_pin2, HIGH);
    analogWrite(arm_pwm, pwm);
//  } else {
//    armMotorPause();
//    return;
//  }
}

void armMotorPause() {
  digitalWrite(arm_motor_pin1, LOW);
  digitalWrite(arm_motor_pin2, LOW);
  analogWrite(arm_pwm, 0);
}

bool check_limit_up() {
  if (digitalRead(limit_up) == LOW) {
    allow_up = true;
    Serial.println("live ur dreams");
  } else {
    allow_up = false;
    Serial.println("nah fam");
  }
  return allow_up;
}

bool check_limit_down() {
  if (digitalRead(limit_down) == LOW) {
    allow_down = true;
  } else {
    allow_down = false;
  }
  return allow_down;
}


