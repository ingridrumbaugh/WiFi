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
int bin[] = {000, 1, 10, 11, 100, 101, 110, 111};

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

  while (!Serial)
    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");

}

// Add code to run a motor when a button is pressed
void loop() {

  // MUX
  digitalWrite(en, LOW); // enable all the time for low
  nrf_server();
  mux_read();
  nrf_client();

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

void mux_read() {

  for (int cnt = 0; cnt < 8; cnt ++) {
    int row = bin[cnt];
    c0 = bitRead(row, 0);
    c1 = bitRead(row, 1);
    c2 = bitRead(row, 2);

    digitalWrite(s0, c0);
    digitalWrite(s1, c1);
    digitalWrite(s2, c2);

    Serial.println("-----------------");
    Serial.print("Accessing Item @ Pin: ");
    Serial.print(cnt);
    Serial.print("  ");
    Serial.print("Value @ This Item is: ");
    Serial.print(analogRead(sig));
    Serial.println("  ");
    Serial.println("-----------------");
    if (cnt == 1) {
      joy1(sig); 
    } else if (cnt == 2) {
      joy2(sig); 
    } else if (cnt == 3) {
      button1(sig); 
    } else if (cnt == 4) {
      button2(sig);  
    } else if (cnt == 5) {
      button3(sig); 
    }
    
    // delay(1000); // time to read - this will probs go down to 100
  }
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







