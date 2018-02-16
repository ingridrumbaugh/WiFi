#include <CD74HC4067.h>
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

#define SWPin 2
#define Joy1 0 // analog read 
#define Joy2 1

WiFiUDP Client;

byte serdata = 0;
byte fromserver = 0;
int val1 = 0; 
int val2 = 0;

// Create instance of MUX  
// 2nd-5th arg --> S0-S3
//CD74HC4067 mux(13,5,8,10); 
//const int common_pin = A0; 

void setup() {
  pinMode(SWPin, INPUT);
  digitalWrite(SWPin, HIGH); 
  //pinMode(common_pin, OUTPUT);  
  
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  // ID of the router , Password of the router
  WiFi.begin("Tenda_6AC248", "Hazmat01"); 
  Serial.println();
  Serial.println();
  Serial.print("Wait for WiFi... ");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Client.begin(81);
}

void loop()
{
  //digitalWrite(common_pin, HIGH); 
  const int port = 80;
  //mux.channel(Joy1);
  //val1 = (analogRead(0)/4)-1;
  val1 = (analogRead(Joy1)/4)-1;
  //val2 = (mux.analogRead(Joystick2)/4)-1; 
  udpsend();
  Serial.print("Val");
  Serial.println(val1);
 // Serial.println(serdata);

//  char serverack = Client.parsePacket();
//  Serial.println(serverack);
//  if (serverack)
//  {
//    fromserver = Client.read();
//    Serial.println(fromserver);
//    delay(1000);
//    if (fromserver == serdata)
//    {
//      serdata = serdata + 1;
//    }
//    else
//    {
//      Serial.println("not equal");
//      Serial.println(fromserver);
//      udpsend();
//    }
//  }
//  else
//  {
//    Serial.println("no data");
//    delay(1000);
//  }

  //client.stop();
  delay(50);
}

void udpsend() {
  const char ip[] = "192.168.0.100";
  Client.beginPacket(ip, 80);
  Client.write(val1);
  Client.endPacket();
  delay(200);
  return;
}


