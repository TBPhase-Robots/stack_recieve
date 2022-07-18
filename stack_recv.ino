//  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

#include <M5Core2.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/node.h>
#include <rclc/subscription.h>
#include <rcutils/strerror.h>
#include <micro_ros_utilities/type_utilities.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <geometry_msgs/msg/vector3.h>
#include <lifecycle_msgs/msg/state.h>
#include <lifecycle_msgs/srv/get_state.h>

#include <Wire.h>           // i2c to connect to IR communication board.

//  ROS error handlers. Calls error_loop if check fails.
#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){char message[128];sprintf(message, "Error on line %d with status %d. Aborting.\n", __LINE__, (int)temp_rc);M5.lcd.println(message);error_loop();}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){char message[128];sprintf(message, "Error on line %d with status %d. Continuing.\n", __LINE__, (int)temp_rc);M5.lcd.println(message);}}


#define ROBOT_I2C_ADDR  8

//  Stack screen width + height
#define WIDTH 320
#define HEIGHT 240

#define MAX_HANDLES 10

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

rcl_subscription_t marker_subscriber;
std_msgs__msg__Int64 marker_msg;

// rcl_client_t registration_client;
// lifecycle_msgs__srv__GetState_Request registration_req;
// lifecycle_msgs__srv__GetState_Response registration_res;
rcl_publisher_t register_publisher;
std_msgs__msg__Int32 register_msg;
rcl_subscription_t id_subscription;
std_msgs__msg__Int32 id_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t setup_node;
rcl_node_t node;
rclc_executor_t setup_executor;
rclc_executor_t executor;

int id = -1;
bool configured = false;

//  Stops and prints an error.
void error_loop(){
  while(1){
    delay(25565);
  }
}

//  Draws a thingy marker to the screen
void drawMarker(u_int64_t data) {
  M5.lcd.clear();

  int size = HEIGHT / 10;
  int side_inset = (WIDTH - HEIGHT) / 2;

  M5.lcd.fillRect(0, 0, WIDTH, size, WHITE);
  M5.lcd.fillRect(0, 0, size + side_inset, HEIGHT, WHITE);

  M5.lcd.fillRect(0, HEIGHT - size, WIDTH, size, WHITE);
  M5.lcd.fillRect(WIDTH - size - side_inset, 0, size + side_inset, HEIGHT, WHITE);

  for (u_int64_t i = 0; i < 36; i++) {
    bool white = (data & ((u_int64_t)1 << i)) != 0;

    int x = side_inset + (i % 6 + 2) * size;
    int y = (i / 6 + 2) * size;
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

  // //  Prints message to the screen
  // M5.lcd.clear();
  // M5.lcd.drawString(s, 0, 0);

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

// Handles marker messages recieved from a ROS subscription
void marker_callback(const void * msgin)
{
  //  Cast received message to int
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;

  //  Draws the marker to the screen
  drawMarker(msg->data);
}

void id_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  id = msg->data;

  char s[32];
  sprintf(s, "Received id: %d", id);
  Serial.println(s);
}

void configure_robot() {
  Serial.println("Removing setup ROS node.");
  RCCHECK(rclc_executor_remove_subscription(&setup_executor, &id_subscription));
  RCCHECK(rcl_publisher_fini(&register_publisher, &setup_node));
  RCCHECK(rcl_subscription_fini(&id_subscription, &setup_node));
  RCCHECK(rcl_node_fini(&setup_node));
  RCCHECK(rclc_executor_fini(&setup_executor));

  delay(500);
  
  Serial.println("Initialising unique ROS node.");

  char node_name[32];
  sprintf(node_name, "robot%d", id);

  //  Create a ROS node
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

  //  Counts the number of handles (subscriptions, timers, etc) being used
  size_t handle_count = 0;

  RCCHECK(rclc_publisher_init_default(
    &register_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/setup/register"));
  handle_count++;

  //  Subscribe to the vector ROS topic, using Vector3 messages
  char vector_topic_name[32];
  sprintf(vector_topic_name, "/robot%d/vectors", id);
  RCCHECK(rclc_subscription_init_default(
    &vector_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    vector_topic_name));
  handle_count++;

  //  Subscribe to the marker ROS topic, using Int64 messages
  char marker_topic_name[32];
  sprintf(marker_topic_name, "/robot%d/markers", id);
  RCCHECK(rclc_subscription_init_default(
    &marker_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    marker_topic_name));
  handle_count++;

  RCCHECK(rclc_executor_init(&executor, &support.context, handle_count, &allocator));

  //  Adds the vector subscription to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &vector_subscriber, &vector_msg,
    &vector_callback, ON_NEW_DATA));
  
  //  Adds the marker subscription to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &marker_subscriber, &marker_msg,
    &marker_callback, ON_NEW_DATA));

  delay(500);

  Serial.println("Sending id acknowledgement.");
  register_msg.data = id;
  RCCHECK(rcl_publish(&register_publisher, &register_msg, NULL));
}

void setup() {
  //  Set up stack
  M5.begin();

  //  Set up serial connection for debugging
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  //  Set up wire to communicate with 3Pi
  Wire.begin();

  // //  Draw checkerboard marker to the screen
  // drawMarker(36805402480);

  Serial.println("Connecting to WiFi.");
  //  Connect to micro ROS agent
  set_microros_wifi_transports("TP-Link_102C", "35811152", "192.168.0.230", 8888);

  //  Wait a bit, not sure why this is needed but its in the ROS tutorial
  delay(500);

  Serial.println("WiFi connected. Initialising setup ROS node.");
  //  Set up ROS setup stuff
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //  Create a ROS node
  RCCHECK(rclc_node_init_default(&setup_node, "temporary_robot_setup_node", "", &support));


  // handle_count++;
  // handle_count++;

  RCCHECK(rclc_publisher_init_default(
    &register_publisher,
    &setup_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/setup/register"));

  RCCHECK(rclc_subscription_init_default(
    &id_subscription,
    &setup_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/setup/ids"));

  //  Creates an executor to handle the subscriptions
  RCCHECK(rclc_executor_init(&setup_executor, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &setup_executor, &id_subscription, &id_msg,
    &id_callback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_client(
  //   &executor, &registration_client, &registration_res,
  //   &registration_callback));


  // RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // lifecycle_msgs__srv__GetState_Request__init(&registration_req);
  // int64_t sequence_number;
  // delay(10);
  // RCCHECK(rcl_send_request(&registration_client, &registration_req, &sequence_number));
  delay(500);
  Serial.println("ROS initialised. Requesting id.");
  register_msg.data = -1;
  RCCHECK(rcl_publish(&register_publisher, &register_msg, NULL));
}

void loop() {
  //  Checks for messages from the subscriptions
  if (!configured) {
    RCCHECK(rclc_executor_spin_some(&setup_executor, RCL_MS_TO_NS(100)));
    if (id != -1) {
      configure_robot();
      configured = true;
    }
  }
  else {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }

}

void printRXStatus() {
  Serial.println( i2c_status_rx.x ); 
  Serial.println( i2c_status_rx.y );
  Serial.println( i2c_status_rx.theta );
  Serial.println( i2c_status_rx.status );
}
