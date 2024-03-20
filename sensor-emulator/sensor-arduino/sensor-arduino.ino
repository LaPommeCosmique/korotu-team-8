#include <ArduinoBLE.h>
BLEService newService("aa818b3b-7033-4bc2-852c-80db2bbe1355"); // creating the service

BLEUnsignedCharCharacteristic charReceivedFromDrone("aa818b3b-7033-4bc2-852c-80db2bbe1356", BLERead | BLENotify);
BLELongCharacteristic millisReceivedFromDrone("aa818b3b-7033-4bc2-852c-80db2bbe1358", BLERead | BLENotify);

BLEUnsignedCharCharacteristic charSentToDrone("aa818b3b-7033-4bc2-852c-80db2bbe1359", BLERead | BLENotify);
BLELongCharacteristic millisSentToDrone("aa818b3b-7033-4bc2-852c-80db2bbe1361", BLERead | BLENotify);

BLEUnsignedCharCharacteristic charToSendToDrone("aa818b3b-7033-4bc2-852c-80db2bbe1362", BLEWrite);


const int ledPin = 2;
long previousMillis = 0;


void setup() {
  Serial.begin(115200);    // initialize serial communication
  while (!Serial);       //starts the program if we open the serial monitor.

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  pinMode(ledPin, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  //initialize ArduinoBLE library
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }

  BLE.setLocalName("Sensor Faker"); //Setting a name that will appear when scanning for Bluetooth® devices
  BLE.setAdvertisedService(newService);

  newService.addCharacteristic(charReceivedFromDrone); //add characteristics to a service
  newService.addCharacteristic(millisReceivedFromDrone); //add characteristics to a service
  newService.addCharacteristic(charSentToDrone); //add characteristics to a service
  newService.addCharacteristic(millisSentToDrone); //add characteristics to a service
  newService.addCharacteristic(charToSendToDrone); //add characteristics to a service

  BLE.addService(newService);  // adding the service

  charReceivedFromDrone.writeValue(0); //set initial value for characteristics
  millisReceivedFromDrone.writeValue(0);
  charSentToDrone.writeValue(0);
  millisSentToDrone.writeValue(0);
  charToSendToDrone.writeValue(0);

  BLE.advertise(); //start advertising the service
  //Serial.println(" Bluetooth® device active, waiting for connections...");

  charToSendToDrone.setEventHandler(BLEWritten, sendCharToDrone);
}

void loop() {
  
  BLEDevice central = BLE.central(); // wait for a Bluetooth® Low Energy central

  if (central) {  // if a central is connected to the peripheral
    //Serial.print("Connected to central: ");
    
    //Serial.println(central.address()); // print the central's BT address
    
    digitalWrite(LED_BUILTIN, HIGH); // turn on the LED to indicate the connection

    // while the central is connected: check serial for communication
    while (central.connected()) {
      if (Serial.available()) {
        char c = Serial.read();
        onReceiveCharFromDrone(c);
      }
    }
    
    digitalWrite(LED_BUILTIN, LOW); // when the central disconnects, turn off the LED
    //Serial.print("Disconnected from central: ");
    //Serial.println(central.address());
  }
}

void sendCharToDrone(BLEDevice central, BLECharacteristic characteristic) {
  char c = (char)(*characteristic.value());
  //Serial.println("Recieved: " + c);
  Serial.print(c);
  millisSentToDrone.writeValue(millis());
  charSentToDrone.writeValue(c);
}

void onReceiveCharFromDrone(char c){
  millisReceivedFromDrone.writeValue(millis());
  charReceivedFromDrone.writeValue(c);
}

