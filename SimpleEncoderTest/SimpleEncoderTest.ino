// Simple Encoder Test: using one interrupt pin 
// Ingrid Rumbaugh 
// 3/7/2018
// Note:
// Each integer is 1/12th of a turn 

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

// values for calculating
// encoder 1 position
int val1;
int enc1Pos = 0;
int enc1ALast = LOW;
int n1 = LOW;

// values for calculating
// encoder 2 position
int val2;
int enc2Pos = 0;
int enc2ALast = LOW;
int n2 = LOW;

void setup() {
  // Setup encoder pins
  pinMode(enc1A, INPUT);
  pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT);
  pinMode(enc2B, INPUT);
  Serial.begin(9600);
}

void loop() {
  // read encoder pins
  n2 = digitalRead(enc2A);
  n1 = digitalRead(enc1A);

  // calculate encoder 1
  if ((enc1ALast == LOW) && (n1 == HIGH)) {
    if (digitalRead(enc1B) == LOW) {
      enc1Pos--;
    } else {
      enc1Pos++;
    }

    Serial.println("Enc1: ");
    Serial.print(enc1Pos);
    //Serial.print("/");
    Serial.println(" ");
  }
  // calculate encoder 2
  if ((enc2ALast == LOW) && (n2 == HIGH)) {
    if (digitalRead(enc2B) == LOW) {
      enc2Pos--;
    } else {
      enc2Pos++;
    }
    Serial.println("Enc2: ");
    Serial.print(enc2Pos);
    //Serial.print("/");
    Serial.println(" ");
  }

  // update previous encoder position
  enc1ALast = n1;
  enc2ALast = n2;

}
