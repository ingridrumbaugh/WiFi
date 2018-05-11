// THIS CODE IS FOR AN UNO
#include <SPI.h>
#include <RH_NRF24.h>
#include <RHReliableDatagram.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// NRF PINS
#define CE   8
#define CSN  10
#define SCK  13
#define MOSI 11
#define MISO 12

// Singleton instance of the radio driver
RH_NRF24 nrf24;
RHReliableDatagram manager(nrf24, CLIENT_ADDRESS);

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
int red1state = 0; // 0 is off --- 1 is on 
int red2state = 0; 
int red3state = 0; 
int red4state = 0; 
int greenstate = 0; 

void setup() {
  Serial.begin(9600);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(sigout, INPUT); 

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  manager.setRetries(50);
  manager.setTimeout(5);
  
  if (!manager.init())
    Serial.println("init failed");

  nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);
}

// Add code to run a motor when a button is pressed
void loop() {
  // MUX
  digitalWrite(en, LOW); // enable all the time for now
  mux_read();
  sendMessage(); 
  receiveMessage();
}

void mux_read() {
  for (int cnt = 0; cnt < 10; cnt ++) {
 
    c0 = bitRead(cnt, 0);
    c1 = bitRead(cnt, 1);
    c2 = bitRead(cnt, 2);
    c3 = bitRead(cnt, 3); 

    digitalWrite(s0, c0);
    digitalWrite(s1, c1);
    digitalWrite(s2, c2);
    digitalWrite(s3, c3); 
    
    if (cnt == 0) {
      pinMode(sigout, INPUT); 
      joystate1 = analogRead(1); 
      Serial.println("Joy 1: ");
      Serial.print(joystate1);
      Serial.println(" ");
    }
    else if (cnt == 1) {
      pinMode(sigout, INPUT); 
      joystate2 = 1023 - analogRead(0); 
      Serial.println("Joy 2: ");
      Serial.print(joystate2);
      Serial.println(" ");
    } 

    //Buttons are swapped to make sense!
    //The board order doesn't make sense!
    //Thanks SOHRAB

    //6 -> 1
    //4 -> 2
    //3 -> 3
    //1 -> 4
    //8 -> 5
    //2 -> 6
    //7 -> 7
    //5 -> 8

    else if (cnt == 7)
      bstate[0] = digitalRead(sigout); 
    else if (cnt == 5)
      bstate[1] = digitalRead(sigout); 
    else if (cnt == 4)
      bstate[2] = digitalRead(sigout); 
    else if (cnt == 2)
      bstate[3] = digitalRead(sigout); 
    else if (cnt == 9)
      bstate[4] = digitalRead(sigout); 
    else if (cnt == 3)
      bstate[5] = digitalRead(sigout); 
    else if (cnt == 8)
      bstate[6] = digitalRead(sigout); 
    else if (cnt == 6)
      bstate[7] = digitalRead(sigout); 
    

//    delay(300); // time to read - for debugging
  }

  for(int i = 0; i < 9; i++)
  {
      Serial.print("Button ");
      Serial.print(i+1);
      Serial.print(": ");
       
      Serial.print(bstate[i]); 
      Serial.println("  ");
  }
}

void sendMessage() {
    //Announce Send
  Serial.println("Sending to ROBOT");

  //Create Message
  byte mess[3];

  uint8_t buttons = 0;

  for(int i = 0; i < 8; i++)
  {
    if(bstate != 0)
    {
      buttons += bstate[i] << i;
    }
  }

  mess[0] = (byte)(joystate1 >> 2);
  mess[1] = (byte)(joystate2 >> 2);
  mess[2] = (byte)buttons;

  //Send Message
  if (manager.sendtoWait(mess, sizeof(mess), SERVER_ADDRESS))
    Serial.println("Message Sent.");
  else
    Serial.println("sendtoWait failed.");
    
  delay(50);
}


void receiveMessage() {
    // Now wait for a reply from the server
  byte buf[1] = {};
  uint8_t len = sizeof(buf);
  uint8_t from;  
  
  if (manager.recvfromAckTimeout((byte*)buf, &len, 10, &from))
  {
    Serial.println("Got reply.");
  }
  else
  {
    Serial.println("No reply.");
  }
  
  for(int j = 0; j < sizeof(buf); j++) 
  {
    Serial.print("Received over transceiver: ");
    Serial.println((int)buf[j]);
  }
}




/**
 *  else if (cnt == 13) {
      bstate[5] = digitalRead(sigout); 
      Serial.print("Button 6: ");
      Serial.print(bstate[5]); 
      Serial.println("  "); 
    } else if (cnt == 14) {
      pinMode(sigout, OUTPUT); 
      if (red1state == 0) {
        digitalWrite(sigout, LOW); 
      } else if (red1state == 1) {
        digitalWrite(sigout, HIGH); 
      }
      
    } else if (cnt == 9) {
      pinMode(sigout, OUTPUT); 
      if (red2state == 0) {
        digitalWrite(sigout, LOW); 
      } else if (red2state == 1) {
        digitalWrite(sigout, HIGH); 
      }
      
    } else if (cnt == 10) {
      pinMode(sigout, OUTPUT); 
      if (red3state == 0) {
        digitalWrite(sigout, LOW); 
      } else if (red3state == 1) {
        digitalWrite(sigout, HIGH); 
      }
      
    } else if (cnt == 11) {
      pinMode(sigout, OUTPUT); 
      if (red4state == 0) {
        digitalWrite(sigout, LOW); 
      } else if (red4state == 1) {
        digitalWrite(sigout, HIGH); 
      }
       
    } else if (cnt == 12) {
      pinMode(sigout, OUTPUT); 
      if (greenstate == 0) {
        digitalWrite(sigout, LOW); 
      } else if (greenstate == 1) {
        digitalWrite(sigout, HIGH); 
      }
 */







