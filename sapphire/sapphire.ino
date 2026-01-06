#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

unsigned long lastRecvTime = 0;

struct PacketData
{
  byte lxAxisValue;
  byte lyAxisValue;
  byte xAxisValue;
  byte yAxisValue;
 
  byte switch1Value;
  byte switch2Value;
  byte switch3Value;
  byte switch4Value;  
  byte switch5Value;
  byte switch6Value;
};
PacketData receiverData;
Servo servo1;     
Servo servo2;
Servo servo3; 

const int B = 33;
const int G = 32;
const int R = 14;
const int led[] = {15,13,12,21,5};
int num = 5;
int enableRightMotor=22; 
int rightMotorPin1=16;
int rightMotorPin2=17;
//Left motor
int enableLeftMotor=23;
int leftMotorPin1=18;
int leftMotorPin2=19;

#define MAX_MOTOR_SPEED 200
const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

void setInputDefaultValues()
{
  // The middle position for joystick. (254/2=127)
  receiverData.lxAxisValue = 127;
  receiverData.lyAxisValue = 127;
  receiverData.xAxisValue = 127;
  receiverData.yAxisValue = 127;
  receiverData.switch1Value = LOW;
  receiverData.switch2Value = LOW;
  receiverData.switch3Value = HIGH;
  receiverData.switch4Value = LOW; 
  receiverData.switch5Value = LOW; 
  receiverData.switch6Value = HIGH;   
}

void mapAndWriteValues()
{
  servo1.write(map(receiverData.lxAxisValue, 0, 254, 60, 120));
  servo2.write(map(receiverData.lyAxisValue, 254, 0, 70, 110));
  int throttle = map( receiverData.xAxisValue, 254, 0, -255, 255);
  int steering = map( receiverData.yAxisValue, 0, 254, -255, 255);  
  int motorDirection = 1;
  
  if (throttle < 0)       //Move car backward
  {
    motorDirection = -1;    
  }

  int rightMotorSpeed, leftMotorSpeed;
  rightMotorSpeed =  abs(throttle) - steering;
  leftMotorSpeed =  abs(throttle) + steering;
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

  rotateMotor(rightMotorSpeed * motorDirection, leftMotorSpeed * motorDirection);
  digitalWrite(led[0], receiverData.switch3Value); digitalWrite(4, receiverData.switch3Value);
  digitalWrite(2, receiverData.switch6Value);
  if(receiverData.switch1Value){digitalWrite(led[1], !receiverData.switch1Value);analogWrite(B,255);analogWrite(R,0);analogWrite(G,0);}else{digitalWrite(led[1],HIGH);}
  if(receiverData.switch2Value){digitalWrite(led[2], !receiverData.switch2Value);analogWrite(B,0);analogWrite(R,0);analogWrite(G,255);}else{digitalWrite(led[2],HIGH);}
  if(receiverData.switch4Value){digitalWrite(led[3], !receiverData.switch4Value);analogWrite(B,0);analogWrite(R,255);analogWrite(G,0);}else{digitalWrite(led[3],HIGH);}
  if(receiverData.switch5Value){digitalWrite(led[4], !receiverData.switch5Value);servo3.write(random(60,120));}else{digitalWrite(led[4],HIGH);}


}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }
  
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  } 

  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));    
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  mapAndWriteValues();  
  lastRecvTime = millis(); 
}

void setUpPinModes()
{
  servo1.attach(27);
  servo2.attach(26);
  servo3.attach(25);
  pinMode(B,OUTPUT);
  pinMode(R,OUTPUT);
  pinMode(G,OUTPUT);
  for(int i = 0;i<num;i++){pinMode(led[i],OUTPUT);}
  pinMode(4,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);  
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel); 
  rotateMotor(0, 0);
  setInputDefaultValues();
  mapAndWriteValues();
}

void setup() 
{
  setUpPinModes();
 
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() 
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    setInputDefaultValues();
    mapAndWriteValues(); 
    rotateMotor(0, 0);
    analogWrite(B,255);analogWrite(R,0);analogWrite(G,0);
    for(int i = 0;i<num;i++){pinMode(led[i],HIGH);}
  }
}
