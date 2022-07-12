#include <M5Core2.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>

#include <Wire.h>           // i2c to connect to IR communication board.

//  ROS error handlers. Calls error_loop if check fails.
#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#define ROBOT_I2C_ADDR  8

//  Stack screen width + height
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

//  Data sent to and from the 3Pi robot
i2c_status_t i2c_status_tx;
i2c_status_t i2c_status_rx;

rcl_subscription_t vector_subscriber;
geometry_msgs__msg__Vector3 vector_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

//  Stops and prints an error.
void error_loop(){
  M5.lcd.println("\nFatal error.");
  while(1){
    delay(25565);
  }
}

//  Draws a thingy marker to the screen
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

// Handles vector messages recieved from a ROS subscription
void vector_callback(const void * msgin)
{
  //  Cast received message to vector
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;

  //  Converts message to a string
  char s[32];
  sprintf(s, "Received: %f\0", msg->x);

  //  Prints message to the screen
  M5.lcd.clear();
  M5.lcd.drawString(s, 0, 0);

  //  Converts message to i2c_status
  i2c_status_tx.x = msg->x;
  i2c_status_tx.y = 0;
  i2c_status_tx.theta = 0;
  i2c_status_tx.status = 0;

  //  Sends i2c_status to the 3Pi
  Wire.beginTransmission(ROBOT_I2C_ADDR);
  Wire.write((uint8_t*)&i2c_status_tx, sizeof(i2c_status_tx));
  Wire.endTransmission();
}

void setup() {
  //  Set up stack
  M5.begin();

  //  Set up serial connection for debugging
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  //  Set up wire to communicate with 3Pi
  Wire.begin();

  //  Draw checkerboard marker to the screen
  drawMarker(0x000000056A56A56A);

  //  Connect to micro ROS agent
  set_microros_wifi_transports("TP-Link_102C", "35811152", "192.168.0.101", 8888);

  //  Wait a bit, not sure why this is needed but its in the ROS tutorial
  delay(2000);

  //  Set up ROS setup stuff
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //  Create a ROS node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node_2", "", &support));

  //  Counts the number of handles (subscriptions, timers, etc) being used
  size_t handle_count = 0;

  //  Subscribe to the vector ROS topic, using Vector3 messages
  RCCHECK(rclc_subscription_init_best_effort(
    &vector_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "vectors"));
  handle_count++;

  //  Creates an executor to handle the subscriptions
  RCCHECK(rclc_executor_init(&executor, &support.context, handle_count, &allocator));

  //  Adds the vector subscription to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &vector_subscriber, &vector_msg,
    &vector_callback, ON_NEW_DATA));
}

void loop() {
  //  Checks for messages from the subscriptions
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void printRXStatus() {
  Serial.println( i2c_status_rx.x ); 
  Serial.println( i2c_status_rx.y );
  Serial.println( i2c_status_rx.theta );
  Serial.println( i2c_status_rx.status );
}
