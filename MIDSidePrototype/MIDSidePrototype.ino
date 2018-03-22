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

int c0 = 0;
int c1 = 0;
int c2 = 0;
int c3 = 0;

int count = 0; // which Y pin is being selected
int bin[] = {000, 1, 10, 11, 100, 101, 110, 111, 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111};

// JOYSTICK & BUTTON PINS
#define joy1SW 7 // this used to be 6
#define joy0SW 9 // this used to be 7

int joystate1 = 0;
int joystate2 = 0;
int bstate1 = 0;
int bstate2 = 0;
int bstate3 = 0; 
int bstate4 = 0;
int bstate5 = 0; 
int bstate6 = 0; 

void setup() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(en, OUTPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  Serial.begin(9600);

}

// Add code to run a motor when a button is pressed
void loop() {
  
  // MUX
  digitalWrite(en, LOW); // enable all the time for low
  mux_read();

}



void mux_read() {

  for (int cnt = 0; cnt < 15; cnt ++) {
    int row = bin[cnt];
    c0 = bitRead(row, 0);
    c1 = bitRead(row, 1);
    c2 = bitRead(row, 2);
    c3 = bitRead(row, 3); 

//    digitalWrite(s0, c0);
//    digitalWrite(s1, c1);
//    digitalWrite(s2, c2);
//    digitalWrite(s3, c3); 

    Serial.println("-----------------");
    Serial.print("Accessing Item @ Pin: ");
    Serial.print(cnt);
    Serial.print("  ");
    Serial.print("Value @ This Item is: ");
    Serial.print(analogRead(sig));
    Serial.println("  ");
    Serial.println("-----------------");
    if (cnt == 0) {
      joy1(sig); 
    } else if (cnt == 1) {
      joy2(sig); 
    } else if (cnt == 2) {
      button1(sig); 
    } else if (cnt == 3) {
      button2(sig);  
    } else if (cnt == 4) {
      button3(sig); 
    } else if (cnt == 5) {
      button4(sig); 
    } else if (cnt == 6) {
      button5(sig);
    } else if (cnt == 7) {
      button6(sig); 
    } else if (cnt == 8) {
      out8();
      LED1(sig);
    } else if (cnt == 9) {
      out9();
      LED2(sig); 
    } else if (cnt == 10) {
      out10();
      LED3(sig); 
    } else if (cnt == 11) {
     out11();
      LED4(sig); 
    } else if (cnt == 12) {
      out12();
      LED5(sig); 
    } 
    
    delay(1000); // time to read - this will probs go down to 100
  }
}

void out8() {
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW); 
  digitalWrite(s2, LOW); 
  digitalWrite(s3, HIGH);
}

void out9() {
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH); 
}

void out10() {
  digitalWrite(s0, LOW);
  digitalWrite(s1, HIGH); 
  digitalWrite(s2, LOW); 
  digitalWrite(s3, HIGH); 
}

void out11() {
  digitalWrite(s0, HIGH); 
  digitalWrite(s1, HIGH);
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH); 
}

void out12() {
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH); 
}

// CHECK BUTTON1 STATE 
// IF IN STOP MODE, DON'T DRIVE MOTORS 
void joy1(int state) {
  joystate1 = state; 
  // send this to the robot 
}

void joy2(int state){
  joystate2 = state; 
  // send this to the robot 
  
}

// STOP BUTTON 
// REVERSES BUTTON STATE - EITHER STOP OR START DRIVE MOTORS 
void button1(int state) {
  bstate1 = state; 
  
}

// ROTATE ARM DC MOTOR 
void button2(int state) {
  bstate2 = state;

  
}

// ROTATE ARM STEPPER MOTOR 
void button3(int state) {
  bstate3 = state; 
  
}

void button4(int state) {
  
}

void button5(int state) {
  
}

void button6(int state) {
  
}

void LED1(int state) {
  analogWrite(sig, state); 
  
}

void LED2(int state) {
  
}

void LED3(int state) {
  
}

void LED4(int state) {
  
}
 
void LED5(int state) {
  
}




