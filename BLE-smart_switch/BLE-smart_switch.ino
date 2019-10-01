#include <ArduinoBLE.h>
#include <SPI.h>

#define LED_PIN     3
#define BUTTON_PIN  4



int currentState;
int debounceState;
int switchState = 0;   
int ledState = 0;

// create service
BLEService lightswitch("FF10");

// create switch characteristic
BLECharCharacteristic switchCharacteristic("FF11", BLERead | BLEWrite);
BLEDescriptor switchDescriptor("2901", "Switch");

BLECharCharacteristic stateCharacteristic("FF12", BLENotify);
BLEDescriptor stateDescriptor("2901", "State");

void setup() {
  Serial.begin(9600);
  while(!Serial);
  if (!BLE.begin()){
    Serial.println("starting BLE failed");
    while(1);
  }

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);  
  
  // set advertised local name and service UUID
  BLE.setLocalName("Light Switch");  // Advertised in scan data as part of GAP
  BLE.setDeviceName("Smart Light Switch"); // Advertised in generic access as part of GATT

  // add service and characteristics
  switchCharacteristic.addDescriptor(switchDescriptor);
  lightswitch.addCharacteristic(switchCharacteristic);

  stateCharacteristic.addDescriptor(stateDescriptor);
  lightswitch.addCharacteristic(stateCharacteristic);

  BLE.addService(lightswitch);

  // assign event handler for characteristic
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);
  
  // begin initialization
  BLE.setAdvertisedService(lightswitch);
  BLE.advertise();

  Serial.println(F("Smart Light Switch"));
}

void loop() {
  BLEDevice central = BLE.central();
  if (central){
    Serial.print("Connected to central: ");
    Serial.print(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    while(central.connected()){
    BLE.poll(); 
  
    currentState = digitalRead(BUTTON_PIN);
    delay(10);
    debounceState = digitalRead(BUTTON_PIN);
    
  
    if( currentState == debounceState  ) { 
      if ( currentState != switchState ) { 
  
        if ( currentState == LOW ) { 
          // Button just released
          
        } else { 
          Serial.print(F("Button event: "));
          if ( ledState == 0 ) {
            stateCharacteristic.setValue(1);
            switchCharacteristic.setValue(1);
            digitalWrite(LED_PIN, HIGH);
            ledState = 1;
             Serial.println(F("light on"));
         } else {
            stateCharacteristic.setValue(0);
            switchCharacteristic.setValue(0);
            digitalWrite(LED_PIN, LOW);
            ledState = 0;
             Serial.println(F("light off"));
         }
        }
        switchState = currentState;
      }
    }
  }}
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print(F("Characteristic event: "));
  if (switchCharacteristic.value()) {
    Serial.println(F("light on"));
    digitalWrite(LED_PIN, HIGH);
    ledState = 1;
    stateCharacteristic.setValue(1);  
    
  } else {
    Serial.println(F("light off"));
    digitalWrite(LED_PIN, LOW);
    ledState = 0;
    stateCharacteristic.setValue(0);   
    
  }
}
