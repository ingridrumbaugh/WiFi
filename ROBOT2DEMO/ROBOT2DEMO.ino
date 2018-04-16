//D. Turch 4/5/2018

#include <SPI.h>
#include <RH_NRF24.h>

// Instance of the radio driver
RH_NRF24 nrf24;

// VALUES FOR MID TRANSMISSIONS
int joystate1 = 0;
int joystate2 = 0;
int bstate[8] = {0,0,0,0,0,0,0,0}; 

void setup()
{
  Serial.begin(9600);
  
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
}


void loop()
{
  receiveMessage();
  sendMessage();
}

void sendMessage()
{
  Serial.println("Sending to MID");
  
  byte gas[1];

  uint8_t temp = 101;
  
  //MESS[0] SHOULD BE THE GAS SENSOR DATA
  gas[0] = (byte)temp;
  
  nrf24.send(gas, sizeof(gas));

  nrf24.waitPacketSent();

  return;
}

void receiveMessage()
{
  Serial.println("Receiving from MID");

  //Define Input (Currently 3 bytes)

  byte buf[3] = {};
  uint8_t len = sizeof(buf);

  //Wait for message
  if(nrf24.recv((byte*)buf, &len))
  {
    Serial.print("Got reply.");
  }
  else
  {
    Serial.println("Receive failed.");
    return;
  }

  joystate1 = buf[0];
  joystate2 = buf[1];

  Serial.print("Joystick 1: ");
  Serial.println(joystate1);
  Serial.print("Joystick 2: ");
  Serial.println(joystate2);

  byte bString = buf[2];

  Serial.print("Buttonstring: ");
  Serial.println(bString);
  
  Serial.print("Buttonstate Parsing: ");

  for(int j = 0; j < 8; j++)
  {
    bstate[7-j] = (bString >> (7 - j)) % 2;
    Serial.print(bstate[7-j]);
  }
  
  Serial.println();
}
