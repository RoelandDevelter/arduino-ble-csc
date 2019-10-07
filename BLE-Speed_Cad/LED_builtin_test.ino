#include <ArduinoBLE.h>

const int updateDelay = 1000;
bool light1 = true;

BLEService switchService("12345");
BLEBoolCharacteristic switchCharacteristic("12334", BLERead | BLENotify | BLEWrite);


void setup() {
pinMode(LED_BUILTIN,OUTPUT);

Serial.begin(9600);
while(!Serial);
if(!BLE.begin()){
  Serial.println("Starting BLE failed");
  while(1);
}

BLE.setEventHandler(BLEConnected, bleConnectHandler);
BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);

BLE.setLocalName("Test");
BLE.setDeviceName("Test");

switchService.addCharacteristic(switchCharacteristic);
switchCharacteristic.setEventHandler(BLEWritten, switched1);
BLE.addService(switchService);

switchCharacteristic.writeValue(true);

BLE.setAdvertisedService(switchService);
BLE.advertise();


Serial.println("BLE active, waiting for connection");
}

void loop() {
  if(!BLE.central()){
  }
}

void bleDisconnectHandler(BLEDevice central){
  Serial.println("Disconnected.");

}

void bleConnectHandler(BLEDevice central){
  Serial.println("Connected.");

}

void switched1(BLEDevice central, BLECharacteristic characteristic){

  if (switchCharacteristic.value()) {
    Serial.println("LED on");
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    Serial.println("LED off");
    digitalWrite(LED_BUILTIN, LOW);
  } 
}
