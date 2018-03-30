// THIS CODE IS FOR AN UNO
#include <SPI.h>
#include <RH_NRF24.h>

// NRF PINS
#define CE   8
#define CSN  10
#define SCK  13
#define MOSI 11
#define MISO 12

// Singleton instance of the radio driver
RH_NRF24 nrf24;

// MUX PINS
#define s0 2 // <-- Change these!!!
#define s1 3 // Used to be S0:10, S1:11, S2:12, S3:13
#define s2 4
#define s3 5
#define en 6 // used to be 5
#define sig A0 // this one's the same as before 
#define sigout 9

int c0 = 0;
int c1 = 0;
int c2 = 0;
int c3 = 0;

int count = 0; // which Y pin is being selected
bool gaslevel[] = {0,0,0,0,0}; 

// JOYSTICK & BUTTON PINS

int joystate1 = 0;
int joystate2 = 0;
int bstate[8] = {0,0,0,0,0,0,0,0}; 

void setup() {
  Serial.begin(9600);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(sigout, OUTPUT); 

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
}

// Add code to run a motor when a button is pressed
void loop() {
  
  // MUX
  digitalWrite(en, LOW); // enable all the time for now
  mux_read();
}

void mux_read() {

  for (int cnt = 0; cnt < 15; cnt ++) {
    digitalWrite(sigout, LOW); 
  
    c0 = bitRead(cnt, 0);
    c1 = bitRead(cnt, 1);
    c2 = bitRead(cnt, 2);
    c3 = bitRead(cnt, 3); 

    digitalWrite(s0, c0);
    digitalWrite(s1, c1);
    digitalWrite(s2, c2);
    digitalWrite(s3, c3); 
//
//    Serial.println("-----------------");
//    Serial.print("\"Pin Number\": ");
//    Serial.println(cnt);
//    Serial.print("LED?: ");
//    Serial.println(analogRead(sig));
//    Serial.print("Accessing Item @ Pin: ");
//    Serial.print(cnt);
//    Serial.print("  ");
//    Serial.print("Value @ This Item is: ");
//    Serial.print(analogRead(sig));
//    Serial.println("  ");
//    Serial.println("-----------------");

    digitalWrite(sigout,LOW);
    
    if (cnt == 0) {
      pinMode(sigout, INPUT); 
      joystate1 = analogRead(sig); 
      Serial.println("Joy 1: ");
      Serial.print(joystate1);
      Serial.println(" ");
    } else if (cnt == 1) {
      pinMode(sigout, INPUT); 
      joystate2 = analogRead(sig); 
      Serial.println("Joy 2: ");
      Serial.print(joystate2);
      Serial.println(" ");
    } else if (cnt == 2) {
      Serial.print("Button 1: "); 
      bstate[0] = digitalRead(sigout); 
      Serial.print(bstate[0]); 
      Serial.println("  "); 
    } else if (cnt == 3) {
      bstate[1] = digitalRead(sigout); 
      Serial.print("Button 2: ");
      Serial.print(bstate[1]); 
      Serial.println("  "); 
    } else if (cnt == 4) {
      bstate[2] = digitalRead(sigout); 
      Serial.print("Button 3: ");
      Serial.print(bstate[2]); 
      Serial.println("  "); 
    } else if (cnt == 5) {
      bstate[3] = digitalRead(sigout); 
      Serial.print("Button 4: ");
      Serial.print(bstate[3]); 
      Serial.println("  "); 
    } else if (cnt == 6) {
      bstate[4] = digitalRead(sigout); 
      Serial.print("Button 5: ");
      Serial.print(bstate[4]); 
      Serial.println("  "); 
    } else if (cnt == 7) {
      bstate[5] = digitalRead(sigout); 
      Serial.print("Button 6: ");
      Serial.print(bstate[5]); 
      Serial.println("  "); 
    } else if (cnt == 8) {
      pinMode(sigout, OUTPUT); 
      digitalWrite(sigout, HIGH); 
    } else if (cnt == 9) {
      pinMode(sigout, OUTPUT); 
      digitalWrite(sigout, HIGH);  
    } else if (cnt == 10) {
      pinMode(sigout, OUTPUT); 
      digitalWrite(sigout, HIGH); 
    } else if (cnt == 11) {
      pinMode(sigout, OUTPUT); 
      digitalWrite(sigout, HIGH);  
    } else if (cnt == 12) {
      pinMode(sigout, OUTPUT); 
      digitalWrite(sigout, HIGH); 
    } 

    delay(500); // time to read - for debugging
  }
}

