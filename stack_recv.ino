#include <M5Core2.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int64.h>
#include <geometry_msgs/msg/vector3.h>

#include <Wire.h>           // i2c to connect to IR communication board.

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define ROBOT_I2C_ADDR  8

#define WIDTH 320
#define HEIGHT 240

// Data to send(tx) and receive(rx)
// on the i2c bus.
// Needs to match the master device

#pragma pack(1)
typedef struct i2c_status {
  float x;                  // 4 bytes
  float y;                  // 4 bytes
  float theta;              // 4 bytes
  uint8_t status;           // 1 byte
} i2c_status_t;
#pragma pack()

i2c_status_t i2c_status_tx;
i2c_status_t i2c_status_rx;

rcl_subscription_t subscriber;
rcl_subscription_t vector_subscriber;
std_msgs__msg__Float32 msg;
geometry_msgs__msg__Vector3 vector_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

void error_loop(){
  M5.lcd.println("\nFatal error.");
  while(1){
    delay(25565);
  }
}

void drawMarker(u_int64_t data) {
  int size = HEIGHT / 6;
  int inset = (WIDTH - HEIGHT) / 2;
  for (u_int64_t i = 0; i < 36; i++) {
    bool white = (data & ((u_int64_t)1 << i)) != 0;

    int x = inset + (i % 6) * size;
    int y = (i / 6) * size;
    if (white) {
      M5.lcd.fillRect(x, y, size, size, WHITE);
    }
  }
}

// void (* rclc_subscription_callback_t)(const void *);

// Implementation example:
void subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  char s[32];

  // Process message
  sprintf(s, "Received: %f\0", msg->data);

  M5.lcd.clear();
  M5.lcd.drawString(s, 0, 0);
}

// Implementation example:
void vector_callback(const void * msgin)
{
  // Cast received message to used type
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;

  char s[32];

  // Process message
  sprintf(s, "Received: %f\0", msg->x);

  M5.lcd.clear();
  M5.lcd.drawString(s, 0, 0);

  i2c_status_tx.x = msg->x;
  i2c_status_tx.y = 0;
  i2c_status_tx.theta = 0;
  i2c_status_tx.status = 0;

  Wire.beginTransmission(ROBOT_I2C_ADDR);
  Wire.write( (uint8_t*)&i2c_status_tx, sizeof( i2c_status_tx ));
  Wire.endTransmission();
}

void setup() {
  // put your setup code here, to run once:
  
  //M5.begin(true, false, true, true);
  M5.begin();
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();  
  // drawMarker(0x000000056A56A56A);
  // M5.lcd.println("\nSetting up I2C aaa");

  M5.lcd.drawString("aaaaaaa", 0, 0);

  // M5.lcd.println("\na");

  set_microros_wifi_transports("TP-Link_102C", "35811152", "192.168.0.101", 8888);

  //  M5.lcd.println("b");

  delay(2000);

  allocator = rcl_get_default_allocator();

  //  M5.lcd.println("c");

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //  M5.lcd.println("d");

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node_2", "", &support));

  //  M5.lcd.println("e");

  // create publisher
  // RCCHECK(rclc_subscription_init_best_effort(
  //   &subscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  //   "topic_name"));

  RCCHECK(rclc_subscription_init_best_effort(
    &vector_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "vectors"));

  // msg.data = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // RCCHECK(rclc_executor_add_subscription(
  //   &executor, &subscriber, &msg,
  //   &subscription_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &vector_subscriber, &vector_msg,
    &vector_callback, ON_NEW_DATA));

  // M5.lcd.println("\ndone");
}

void loop() {

  // Setup data to send to robot


  // Send an update down to the robot

  // Read values back from the robot.
  // Serial.println("Read: ");
  // Wire.requestFrom( ROBOT_I2C_ADDR, sizeof( i2c_status_rx ));
  // Wire.readBytes( (uint8_t*)&i2c_status_rx, sizeof( i2c_status_rx ));
  // printRXStatus();

  // RCSOFTCHECK(rcl_publish(&subscriber, &msg, NULL));
  // msg.data++;

  // M5.lcd.println("\ndid");

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  

  // delay(100);
}

void printRXStatus() {
  Serial.println( i2c_status_rx.x ); 
  Serial.println( i2c_status_rx.y );
  Serial.println( i2c_status_rx.theta );
  Serial.println( i2c_status_rx.status );
}
