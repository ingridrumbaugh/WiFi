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

int val; 
int enc1Pos = 0;
int enc1ALast = LOW; 
int n = LOW; 

void setup() {
  pinMode(enc1A, INPUT); 
  pinMode(enc1B, INPUT); 
  Serial.begin(9600); 

}

void loop() {
  n = digitalRead(enc1A); 
  if ((enc1ALast == LOW) && (n == HIGH)) {
    
    if (digitalRead(enc1B) == LOW) {
      
      enc1Pos--;
      
    } else {
      
      enc1Pos++; 
      
    }
    Serial.print(enc1Pos);
    Serial.print("/"); 
  }
  enc1ALast = n; 

}
