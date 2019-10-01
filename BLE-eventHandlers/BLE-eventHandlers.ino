#include <ArduinoBLE.h>


// define BLE parameters
BLEService BattService("180F"); // Battery service UUID
BLEUnsignedIntCharacteristic BattChar("2A19", BLERead | BLENotify);


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  while(!Serial);
  if (!BLE.begin()){
    Serial.println("starting BLE failed");
    while(1);
  }
  
  BLE.setLocalName("Zephyr");
  BLE.setDeviceName("Bicycle Computer");
  
  BattService.addCharacteristic(BattChar);
  
  BLE.addService(BattService);


  BLE.setAdvertisedService(BattService); // add the service UUID
  BLE.advertise();

  Serial.println("Bluetooth active, waiting for connection");

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  
}
  
  void blePeripheralConnectHandler(BLEDevice central) {
    // central connected event handler
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
  }
  
  void blePeripheralDisconnectHandler(BLEDevice central) {
    // central disconnected event handler
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
  }

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();
}
