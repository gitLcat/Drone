#include <micro_ros_arduino.h>
#include <ArduinoBLE.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
rcl_publisher_t sensor_data;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher


  RCCHECK(rclc_publisher_init_default(
    &sensor_data,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "raw_sensor_data"));
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Serial.begin(115200);

  while (!Serial);

  // initialize the BLE hardware

  BLE.begin();

  Serial.println("BLE Central - LED control");

  // start scanning for Button Device BLE peripherals

  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
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

  while (peripheral.connected()) {


      // Gyro Data
        float Gyro[3];
        LEDCharacteristic.readValue(Gyro,12);
      // Gyro X Data
        msg.data = Gyro[0];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));
        // Gyro Y Data
        msg.data = Gyro[1];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));
        // Gyro Z Data
        msg.data = Gyro[2];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));


        float Accel[3];
        AccelerationCharacteristic.readValue(Accel,12);
        // Accel X Data
        msg.data = Accel[0];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));
        // Accel Y Data
        msg.data = Accel[1];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));
        // Accel Z Data
        msg.data = Accel[2];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));

        float Magn[3];
        MagneticFieldCharacteristic.readValue(Magn,12);
        // Magn X Data
        msg.data = Magn[0];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));
        // Magn Y Data
        msg.data = Magn[1];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));
        // Magn Z Data
        msg.data = Magn[2];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));

        float Alti[2];
        //Pressure
        msg.data = Alti[0];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));
        //Altitude
        msg.data = Alti[1];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));

        float Temp[2];
        TemperatureCharacteristic.readValue(Temp,12);
        msg.data = Temp[0];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));
        msg.data = Temp[1];
        RCSOFTCHECK(rcl_publish(&sensor_data,&msg,NULL));
    delay(500);

  }

  Serial.println("Peripheral disconnected");

}
