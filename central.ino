#include <ArduinoBLE.h>


void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

  while (!Serial);

  // initialize the BLE hardware

  BLE.begin();

  Serial.println("BLE Central - LED control");

  // start scanning for Button Device BLE peripherals

  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");

}

void loop() {

  // check if a peripheral has been discovered

  BLEDevice peripheral = BLE.available();

  if (peripheral) {

    // discovered a peripheral, print out address, local name, and advertised service

    Serial.print("Found ");

    Serial.print(peripheral.address());

    Serial.print(" '");

    Serial.print(peripheral.localName());

    Serial.print("' ");

    Serial.print(peripheral.advertisedServiceUuid());

    Serial.println();

    if (peripheral.localName().indexOf("Button Device") < 0) {

      Serial.println("No 'Button Device' in name");

      return;  // If the name doesn't have "Button Device" in it then ignore it

    }

    // stop scanning

    BLE.stopScan();

    controlLed(peripheral);

    // peripheral disconnected, start scanning again

    BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");

  }

}

void controlLed(BLEDevice peripheral) {

  // connect to the peripheral

  Serial.println("Connecting ...");

  if (peripheral.connect()) {

    Serial.println("Connected");

  } else {

    Serial.println("Failed to connect!");

    return;

  }

  // discover peripheral attributes

  Serial.println("Discovering attributes ...");

  if (peripheral.discoverAttributes()) {

    Serial.println("Attributes discovered");

  } else {

    Serial.println("Attribute discovery failed!");

    peripheral.disconnect();

    return;

  }

  // retrieve the LED characteristic

  BLECharacteristic LEDCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");

  if (!LEDCharacteristic) {

    Serial.println("Peripheral does not have LED characteristic!");

    peripheral.disconnect();

    return;

  }

  while (peripheral.connected()) {


      
        float oData[3];
        LEDCharacteristic.readValue(oData,12);

        Serial.print("\t\tHeading: ");
        Serial.print(oData[0]);
        Serial.println('\t');

        Serial.print("\t\tPitch: ");
        Serial.print(oData[1]);
        Serial.println('\t');

        Serial.print("\t\tRoll: ");
        Serial.print(oData[2]);
        Serial.println('\t');
      

      

       

    
    delay(500);

  }

  Serial.println("Peripheral disconnected");

}