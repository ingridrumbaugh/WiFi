// #include <SPI.h> 

#define pwm1 3
#define pwm2 4

#define arm_pwm1 51
#define arm_pwm2 47

#define motor1Pin1 7 // INA
#define motor1Pin2 8 // INB

#define motor2Pin1 5
#define motor2Pin2 6

#define arm_motor1_pin1 52
#define arm_motor1_pin2 53

#define arm_motor2_pin1 49
#define arm_motor2_pin2 48

void setup() {
  pinMode(pwm1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT); 
  pinMode(motor1Pin2, OUTPUT); 

  pinMode(pwm2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT); 
  
  Serial.begin(9600); 
}

void loop() {
  motor1Forward(100); 
  delay(1000); 
  motor1Backward(100); 
  delay(1000); 
  motor1Pause(); 
  delay(1000); 
  motor2Forward(100); 
  delay(1000);
  motor2Backward(100); 
  delay(1000); 
  motor2Pause();
  delay(1000);
}

void motor1Forward(int pwm) {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  analogWrite(pwm1, pwm); 
}

void motor1Backward(int pwm) {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  analogWrite(pwm1, pwm);
}

void motor1Pause() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, HIGH); 
  analogWrite(pwm1, 0); 
}

void motor2Forward(int pwm) {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(pwm2, pwm); 
}

void motor2Backward(int pwm) {
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 
  analogWrite(pwm2, pwm); 
}

void motor2Pause() {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, HIGH); 
  analogWrite(pwm2, 0); 
}





