/*
  UDPSendReceiveString:
  This sketch receives UDP message strings, prints them to the serial port
  and sends an "acknowledge" string back to the sender

  A Processing sketch is included at the end of file that can be used to send
  and received messages for testing with a computer.

  created 21 Aug 2010
  by Michael Margolis

  This code is in the public domain.
*/


#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

const int pwm = 3;
const int motorPin1 = 7;
const int motorPin2 = 8;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 0, 100);

unsigned int localPort = 80;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

void setup() {
  pinMode(pwm, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  // start the Ethernet and UDP:
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  Serial.begin(9600);
  //Serial.println("Anything");
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    //int a = packetBuffer;
    int b = packetBuffer[0];
    //Serial.println(a);
    Serial.println(b);
    if (b < 20) {
      Serial.println("LOW, B = ");
      Serial.print(b); 
      Serial.println("");
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      analogWrite(pwm, 100);
    }
    else if (b > 200 && b < 300) {
      Serial.println("HIGH, B = "); 
      Serial.print(b); 
      Serial.println(""); 
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      analogWrite(pwm, 100); 
    } else if (b > 100 && b < 200) {
      Serial.println("Motor should be stopped now."); 
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, HIGH);
      analogWrite(pwm, 0); 
    } 
  }
  delay(10);
}
