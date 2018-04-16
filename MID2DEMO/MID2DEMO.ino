#include <SPI.h>
#include <RH_NRF24.h>

// Singleton instance of the radio driver
RH_NRF24 nrf24;

// VALUES FOR MID TRANSMISSIONS
int joystate1 = 122;
int joystate2 = 123;
int bstate[8] = {0,1,0,1,0,1,0,1}; 

void setup() {
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
  sendMessage();
  receiveMessage();
}

void sendMessage()
{
  Serial.println("Sending to ROBOT");
  
  byte mess[3];

  uint8_t buttons = 0;

  for(int i = 0; i < 8; i++)
  {
    if(bstate != 0)
    {
      buttons += bstate[i] << i;
    }
  }

  mess[0] = (byte)joystate1;
  mess[1] = (byte)joystate2;
  mess[2] = (byte)buttons;
  
  nrf24.send(mess, sizeof(mess));

  nrf24.waitPacketSent();

  return;
}

void receiveMessage()
{
  Serial.println("Receiving from ROBOT");

  //Define Input (Currently 1 byte)

  byte buf[1] = {};
  uint8_t len = sizeof(buf);

  //Wait for message

  if (nrf24.waitAvailableTimeout(500))
  { 
    // Should be a reply message for us now   
    if (nrf24.recv((byte*)buf, &len))
    {
      Serial.println("Got reply.");
    }
    else
    {
      Serial.println("Receive failed.");
      return;
    }
  }

  for(int j = 0; j < sizeof(buf); j++)
  {
    Serial.print("Received over transceiver: ");
    Serial.println((int)buf[j]);
  }
}
