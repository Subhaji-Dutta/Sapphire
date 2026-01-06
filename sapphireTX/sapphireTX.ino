#include <esp_now.h>
#include <WiFi.h>

#define RX 33
#define RY 34
#define LX 32
#define LY 35

#define Sw1 5 
#define Sw2 16 
#define Sw3 17
#define Sw4 18 
#define Sw5 19
#define Sw6 21

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t receiverMacAddress[] = {0xCC,0x7B,0x5C,0x34,0xA4,0x3C};

struct PacketData
{
  byte lxAxisValue;
  byte lyAxisValue;
  byte rxAxisValue;
  byte ryAxisValue;
 
  byte switch1Value;
  byte switch2Value;
  byte switch3Value;
  byte switch4Value;  
  byte switch5Value;
  byte switch6Value;  
};
PacketData data;

//This function is used to map 0-4095 joystick value to 0-254. hence 127 is the center value which we send.
//It also adjust the deadband in joystick.
//Jotstick values range from 0-4095. But its center value is not always 2047. It is little different.
//So we need to add some deadband to center value. in our case 1800-2200. Any value in this deadband range is mapped to center 127.
int mapAndAdjustJoystickDeadBandValues(int value, bool reverse)
{
  if (value >= 2200)
  {
    value = map(value, 2200, 4095, 127, 254);
  }
  else if (value <= 1800)
  {
    value = (value == 0 ? 0 : map(value, 1800, 0, 127, 0));  
  }
  else
  {
    value = 127;
  }

  if (reverse)
  {
    value = 254 - value;
  }
  Serial.println(value);  
  return value;
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t ");
  Serial.println(status);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("Succes: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Succes: Added peer");
  } 

  pinMode(Sw1,INPUT_PULLUP);
  pinMode(Sw2,INPUT_PULLUP);
  pinMode(Sw3,INPUT_PULLUP);
  pinMode(Sw4,INPUT_PULLUP); 
  pinMode(Sw5,INPUT_PULLUP);
  pinMode(Sw6,INPUT_PULLUP);

     
}
 
void loop() 
{
  data.lxAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(LX), false);
  data.lyAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(LY), false);
  data.rxAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(RX), false);
  data.ryAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(RY), false);
  data.switch1Value   = !digitalRead(Sw1);
  data.switch2Value   = !digitalRead(Sw2);
  data.switch3Value   = !digitalRead(Sw3);
  data.switch4Value   = !digitalRead(Sw4);
  data.switch5Value   = !digitalRead(Sw5);
  data.switch6Value   = digitalRead(Sw6);
 
  
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &data, sizeof(data));
  if (result == ESP_OK) 
  {
    Serial.println("Sent with success");
  }
  else 
  {
    Serial.println("Error sending the data");
  }    
  
  delay(50);
}

