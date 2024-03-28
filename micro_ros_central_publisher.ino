#include <micro_ros_arduino.h>
#include <ArduinoBLE.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/string.h>
rcl_publisher_t test;
rcl_publisher_t gyro_publisher;
rcl_publisher_t Tilt_publisher;
std_msgs__msg__String angle;
std_msgs__msg__Float32 msg;
geometry_msgs__msg__Vector3 g_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

int degreesX = 0;

int degreesY = 0;

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


  RCCHECK(rclc_publisher_init_best_effort(
    &test,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "test"));

  RCCHECK(rclc_publisher_init_best_effort(
  &gyro_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
  "gyro"));

  RCCHECK(rclc_publisher_init_best_effort(
    &Tilt_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "Tilt"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
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


      
        float Gyro[3];
        LEDCharacteristic.readValue(Gyro,12);

        Serial.print("Gyro_X: ");
        Serial.println(Gyro[0]);


        Serial.print("Gyro_Y: ");
        Serial.println(Gyro[1]);


        Serial.print("Gyro_Z: ");
        Serial.println(Gyro[2]);

        g_msg.x = Gyro[0];
        g_msg.y = Gyro[1];
        g_msg.z = Gyro[2];

        RCSOFTCHECK(rcl_publish(&gyro_publisher, &g_msg, NULL));


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

        angle.data.data = "Tilting_up";
    RCSOFTCHECK(rcl_publish(&gyro_publisher, &angle, NULL));


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
        msg.data = 001;
        RCSOFTCHECK(rcl_publish(&test,&msg,NULL));
        msg.data = Temp[0];
        RCSOFTCHECK(rcl_publish(&test,&msg,NULL));
        msg.data = 002;
        RCSOFTCHECK(rcl_publish(&test,&msg,NULL));
        msg.data = Temp[1];
        RCSOFTCHECK(rcl_publish(&test,&msg,NULL));
    delay(500);

  }

  Serial.println("Peripheral disconnected");

}
