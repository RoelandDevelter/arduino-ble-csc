#include <ArduinoBLE.h>

// define pin numbers
const int pin_speed = 2;
const int pin_cad = 3;
const int pin_batt = A1;
const int pin_rear = 4;
const int pin_EL = 5;
const int pin_front = 6;

#pragma pack(1)   // packs everything closer to each other (= get rid of padding), required for proper readout
// define csc measurement packet
typedef struct sensorData_t {
  byte flags;
  uint32_t cumWheelRev;
  uint16_t lastWheelEvent;
  uint16_t cumCrankRev;
  uint16_t lastCrankEvent;
};

#define PACKET_SIZE sizeof(sensorData_t)

typedef union blePacket_t {
  sensorData_t sensor;
  byte blePacket[PACKET_SIZE];
};
blePacket_t cscData;

// define battery parameters
int prevBattLevel = 0;
long previousMillis = 0; // counter for updating battery
const int updateDelay = 5000; // in millis

// define csc parameters
unsigned int dt; // time difference between pulses in millis
unsigned long t0_w = 0; // time at previous wheel pulse in millis
unsigned long t0_c = 0; // time at previous crank pulse in millis
unsigned long t1; // time at current pulse in millis
unsigned long wheel_revs = 0; // total amount of revolutions, measure for total distance
unsigned long crank_revs = 0; // total amount of revolutions, measure for total distance
const int dt_min = 20; // minimal time between pulses to debounce sensor in millis
boolean cscDataFlag = false; // flag to know when data needs to be pushed

// define BLE parameters
BLEService cscService("1816"); // UUID for cycling speed
BLECharacteristic cscMeasurementChar("2A5B", BLENotify, PACKET_SIZE); // characteristic of the computer
BLEUnsignedShortCharacteristic cscFeatureChar("2A5C", BLERead);

BLEService battService("180F"); // Battery service UUID
BLEUnsignedIntCharacteristic battChar("2A19", BLERead | BLENotify);

BLEService lightSwitchService("86e6c5b3-1f4d-4013-a263-502212ca82dd");
BLEBoolCharacteristic switchCharacteristicFront("51ca1f44-70c8-468f-bda8-dba60923a711", BLERead | BLENotify | BLEWrite);
BLEDescriptor descriptorFront("2901", "FrontLight");
BLEBoolCharacteristic switchCharacteristicRear("d1e9f2a5-d311-47c2-b023-9bf246dae93e", BLERead | BLENotify | BLEWrite);
BLEDescriptor descriptorRear("2901", "RearLight");
BLEBoolCharacteristic switchCharacteristicEL("d469b383-4137-42b7-b977-16c8e1b383be", BLERead | BLENotify | BLEWrite);
BLEDescriptor descriptorEL("2901", "ELTape");

void setup() {
  // set up pins
  pinMode(pin_batt, INPUT);
  pinMode(pin_speed, INPUT_PULLUP);
  pinMode(pin_cad, INPUT_PULLUP);
  pinMode(pin_rear, OUTPUT);
  pinMode(pin_front, OUTPUT);
  pinMode(pin_EL, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pin_speed), updateSpeed, FALLING); // use pull-up resistor
  attachInterrupt(digitalPinToInterrupt(pin_cad), updateCad, FALLING); // use pull-up resistor

  // start protocols
  if (!BLE.begin()) {
    while (1);
  }

  // set BLE parameters
  BLE.setLocalName("Zephyr");
  BLE.setDeviceName("Bicycle Computer");
  BLE.setEventHandler(BLEConnected, bleConnectHandler);
  BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);

  // add BLE characteristics & descriptors
  cscService.addCharacteristic(cscMeasurementChar);
  cscService.addCharacteristic(cscFeatureChar);
  battService.addCharacteristic(battChar);
  switchCharacteristicFront.addDescriptor(descriptorFront);
  switchCharacteristicRear.addDescriptor(descriptorRear);
  switchCharacteristicEL.addDescriptor(descriptorEL);
  lightSwitchService.addCharacteristic(switchCharacteristicFront);
  lightSwitchService.addCharacteristic(switchCharacteristicRear);
  lightSwitchService.addCharacteristic(switchCharacteristicEL);
  switchCharacteristicFront.setEventHandler(BLEWritten, switchedFront);
  switchCharacteristicRear.setEventHandler(BLEWritten, switchedRear);
  switchCharacteristicEL.setEventHandler(BLEWritten, switchedEL);

  // add BLE services
  BLE.addService(cscService);
  BLE.addService(battService);
  BLE.addService(lightSwitchService);

  // write initial values
  cscFeatureChar.writeValue(1); // bit0: wheel rev data support, bit1: crank rev data support, bit2: multiple sensor locations
  cscData.sensor.flags = 3; // bit 0: wheel rev data present, bit 1: crank rev data present
  cscData.sensor.cumWheelRev = 0;
  cscData.sensor.lastWheelEvent = 1;
  cscData.sensor.cumCrankRev = 0;
  cscData.sensor.lastCrankEvent = 1;
  cscMeasurementChar.writeValue(cscData.blePacket, PACKET_SIZE);
  switchCharacteristicFront.writeValue(false);
  switchCharacteristicRear.writeValue(false);
  switchCharacteristicEL.writeValue(false);

  // start advertising
  BLE.setAdvertisedService(cscService); // add the service UUID
  BLE.advertise();

}

void loop() {
  BLEDevice central = BLE.central(); // listen for BLE central to connect
  if (central) {
    while (central.connected()) {

      long currentMillis = millis();
      if (currentMillis - previousMillis >= updateDelay) {
        previousMillis = currentMillis;
        updateBattery();
      }
      if (cscDataFlag) {
        cscMeasurementChar.writeValue(cscData.blePacket, PACKET_SIZE);
        cscDataFlag = false;
      }
    }
  }
}

// called every updateDelay
void updateBattery() {
  int currBattLevel = map(analogRead(pin_batt), 0, 1023, 0, 100);
  if (prevBattLevel != currBattLevel) {
    prevBattLevel = currBattLevel;
    battChar.writeValue(currBattLevel);
  }
}

// called when interrupt on wheel event
void updateSpeed() {
  t1 = millis();
  dt = t1 - t0_w;
  if (dt > dt_min) {
    t0_w = t1;
    wheel_revs += 1;

    cscData.sensor.cumWheelRev = wheel_revs;
    cscData.sensor.lastWheelEvent = dt;
    cscDataFlag = true;   // don't write to characteristic in interrupt routine
  }
}

// called when interrupt on crank event
void updateCad() {
  t1 = millis();
  dt = t1 - t0_c;
  if (dt > dt_min) {
    t0_c = t1;
    crank_revs += 1;

    cscData.sensor.cumCrankRev = crank_revs;
    cscData.sensor.lastCrankEvent = dt;
    cscDataFlag = true;   // don't write to characteristic in interrupt routine
  }
}

// called when disconnected from BLE central
void bleDisconnectHandler(BLEDevice central) {
  digitalWrite(LED_BUILTIN, LOW);
}

// called when connected to BLE central
void bleConnectHandler(BLEDevice central) {
  digitalWrite(LED_BUILTIN, HIGH);
}

// called when written to front light switch characteristic
void switchedFront(BLEDevice central, BLECharacteristic characteristic) {
  if (switchCharacteristicFront.value()) {
    digitalWrite(pin_front, HIGH);
  } else {
    digitalWrite(pin_front, LOW);
  }
}

// called when written to rear light switch characteristic
void switchedRear(BLEDevice central, BLECharacteristic characteristic) {
  if (switchCharacteristicRear.value()) {
    digitalWrite(pin_rear, HIGH);
  } else {
    digitalWrite(pin_rear, LOW);
  }
}

// called when written to EL tape switch characteristic
void switchedEL(BLEDevice central, BLECharacteristic characteristic) {
  if (switchCharacteristicEL.value()) {
    digitalWrite(pin_EL, HIGH);
  } else {
    digitalWrite(pin_EL,LOW);
  }
}
