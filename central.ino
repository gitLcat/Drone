#include <ArduinoBLE.h>

int degreesX = 0;

int degreesY = 0;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

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

  BLECharacteristic LEDCharacteristic = peripheral.characteristic("0001");
  BLECharacteristic AccelerationCharacteristic = peripheral.characteristic("0002");
  BLECharacteristic MagneticFieldCharacteristic = peripheral.characteristic("0003");
  BLECharacteristic AltitudeCharacteristic = peripheral.characteristic("0004");
  BLECharacteristic TemperatureCharacteristic = peripheral.characteristic("0005");
  if (!LEDCharacteristic) {

    Serial.println("Peripheral does not have LED characteristic!");

    peripheral.disconnect();

    return;

  }
if (!AccelerationCharacteristic) {

    Serial.println("Peripheral does not have AccelerationCharacteristic!");

    peripheral.disconnect();

    return;

  }

  if (!MagneticFieldCharacteristic) {

    Serial.println("Peripheral does not have MagneticFieldCharacteristic!");

    peripheral.disconnect();

    return;

  }

  if (!AltitudeCharacteristic) {

    Serial.println("Peripheral does not have AltitudeCharacteristic!");

    peripheral.disconnect();

    return;

  }

  if (!TemperatureCharacteristic) {

    Serial.println("Peripheral does not have TemperatureCharacteristic!");

    peripheral.disconnect();

    return;

  }

  while (peripheral.connected()) {


      
        float Gyro[3];
        LEDCharacteristic.readValue(Gyro,12);

        Serial.print("Gyro_X: ");
        Serial.println(Gyro[0]);


        Serial.print("Gyro_Y: ");
        Serial.println(Gyro[1]);


        Serial.print("Gyro_Z: ");
        Serial.println(Gyro[2]);



        float Accel[3];
        AccelerationCharacteristic.readValue(Accel,12);

        Serial.print("Accel_X: ");
        Serial.print(Accel[0]);
        Serial.println(' G');

        Serial.print("Accel_Y: ");
        Serial.print(Accel[1]);
        Serial.println(' G');

        Serial.print("Accel_Z: ");
        Serial.print(Accel[2]);
        Serial.println(' G');

        if(Accel[0] > 0.1){

    Accel[0] = 100*Accel[0];

    degreesX = map(Accel[0], 0, 97, 0, 90);

    Serial.print("Tilting up ");

    Serial.print(degreesX);

    Serial.println(" degrees");

    }

  if(Accel[0] < -0.1){

    Accel[0] = 100*Accel[0];

    degreesX = map(Accel[0], 0, -100, 0, 90);

    Serial.print("Tilting down ");

    Serial.print(degreesX);

    Serial.println(" degrees");

    }

  if(Accel[1] > 0.1){

    Accel[1] = 100*Accel[1];

    degreesY = map(Accel[1], 0, 97, 0, 90);

    Serial.print("Tilting left ");

    Serial.print(degreesY);

    Serial.println(" degrees");

    }

  if(Accel[1] < -0.1){

    Accel[1] = 100*Accel[1];

    degreesY = map(Accel[1], 0, -100, 0, 90);

    Serial.print("Tilting right ");

    Serial.print(degreesY);

    Serial.println(" degrees");

    }

        float Magn[3];
        MagneticFieldCharacteristic.readValue(Magn,12);
        Serial.print("Magn_X: ");
        Serial.println(Magn[0]);

        Serial.print("Magn_Y: ");
        Serial.println(Magn[1]);


        Serial.print("Magn_Z: ");
        Serial.println(Magn[2]);
        

        float Alti[2];
        AltitudeCharacteristic.readValue(Alti,12);
        Serial.print("Pressure : ");
        Serial.print(Alti[0]);
        Serial.println('kPa');

        Serial.print("altitude : ");
        Serial.println(Alti[1]);
        
        float Temp[2];
        TemperatureCharacteristic.readValue(Temp,12);
        Serial.print("Temperature :");
        Serial.print(Temp[0]);
        Serial.println(" °C");

        Serial.print("Humidity :");
        Serial.print(Temp[1]);
        Serial.println(" %");
        Serial.println();
        Serial.println();
    delay(500);

  }

  Serial.println("Peripheral disconnected");

}
