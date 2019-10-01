#include <ArduinoBLE.h>

// define pin numbers
const int pin_km = 2;
const int pin_batt = A0;

#pragma pack(1)   // packs everything closer to each other, required to get 
// define csc measurement packet
typedef struct sensorData_t{
  byte flags;
  uint32_t cumWheelRev;
  uint16_t lastWheelEvent;
};

#define PACKET_SIZE sizeof(sensorData_t)

typedef union BLE_packet_t{
  sensorData_t sensor;
  byte BLE_packet[PACKET_SIZE];
};
BLE_packet_t CSCdata; 


// define battery parameters
int prevBattLevel = 0;
long previousMillis = 0; // counter for updating battery
const int updateDelay = 1000; // in millis


// define csc parameters
unsigned int dt; // time difference between pulses in millis
unsigned long t0 = 0; // time at previous pulse in millis
unsigned long t1; // time at current pulse in millis
unsigned long revs = 0; // total amount of revolutions, measure for total distance
const int dt_min = 20; // minimal time between pulses to debounce sensor in millis
boolean CSCdataFlag = false; // flag to know when data needs to be pushed

// define BLE parameters
BLEService CSCService("1816"); // UUID for cycling speed 
BLECharacteristic CSCMeasurementChar("2A5B", BLENotify, PACKET_SIZE); // characteristic of the computer
BLEUnsignedShortCharacteristic CSCFeatureChar("2A5C", BLERead);

BLEService BattService("180F"); // Battery service UUID
BLEUnsignedIntCharacteristic BattChar("2A19", BLERead | BLENotify);

void setup() {
  pinMode(pin_batt, INPUT);
  pinMode(pin_km, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_km), updateSpeed, FALLING); // use pull-up resistor
  
  Serial.begin(9600);
  while(!Serial);
  if (!BLE.begin()){
    Serial.println("starting BLE failed");
    while(1);
  }
  
  BLE.setLocalName("Zephyr");
  BLE.setDeviceName("Bicycle Computer");
  
  CSCService.addCharacteristic(CSCMeasurementChar);
  CSCService.addCharacteristic(CSCFeatureChar);
  BattService.addCharacteristic(BattChar);
  
  BLE.addService(CSCService);
  BLE.addService(BattService);

  // write initial values
  CSCFeatureChar.writeValue(1); // bit0: wheel rev data support, bit1: crank rev data support, bit2: multiple sensor locations 
  CSCdata.sensor.flags = 1; // bit 0: wheel rev data present, bit 1: crank rev data present
  CSCdata.sensor.cumWheelRev = 0;
  CSCdata.sensor.lastWheelEvent = 1;
  CSCMeasurementChar.writeValue(CSCdata.BLE_packet, PACKET_SIZE);
  
  BLE.setAdvertisedService(CSCService); // add the service UUID
  BLE.advertise();

  Serial.println("Bluetooth active, waiting for connection");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central){
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
  
    while(central.connected()){
      
      long currentMillis = millis();
      if (currentMillis - previousMillis >= updateDelay){
        previousMillis = currentMillis;
        updateBattery();
      }
      if (CSCdataFlag){
        CSCMeasurementChar.writeValue(CSCdata.BLE_packet, PACKET_SIZE);
        CSCdataFlag = false;
      }
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
}

void updateBattery(){
  int currBattLevel = map(analogRead(pin_batt), 0, 1023, 0, 100);
  if (prevBattLevel != currBattLevel){
    prevBattLevel = currBattLevel;
    BattChar.writeValue(currBattLevel);
  }
}

void updateSpeed(){
  t1 = millis();
  dt = t1-t0;
  if (dt>dt_min){
      t0 = t1;
      revs += 1;

      CSCdata.sensor.cumWheelRev = revs;
      CSCdata.sensor.lastWheelEvent = dt;
      CSCdataFlag = true;   // don't write to char in ISP
  }
}
