#include <avr/eeprom.h>

// WRIST MOTOR PINS
#define wrist_pwm 10
#define wrist_motor_pin1 11
#define wrist_motor_pin2 12

// WRIST ENCODERS
#define enc3A 2
#define enc3B 3
bool wrist_moving = false;
bool allow_rotate_cw = false; // for the wrist
bool allow_rotate_ccw = false;
int cw_max, ccw_max;

// VALUES FOR WRIST MOTOR ENCODER
int val3;
int enc3Pos = 0;
int enc3ALast = LOW;
int n3 = LOW;

struct settings_t {
  int arm_enc_val;
} settings;

void setup() {
  pinMode(wrist_pwm, OUTPUT);
  pinMode(wrist_motor_pin1, OUTPUT);
  pinMode(wrist_motor_pin2, OUTPUT);
  
  pinMode(enc3A, INPUT);
  pinMode(enc3B, INPUT);
  
  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));
  enc3Pos = settings.arm_enc_val;

  Serial.begin(9600);
  Serial.println("Saved Encoder Value: " + (int)settings.arm_enc_val);
}

void loop() {
  wrist_clockwise(150);  
}

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



