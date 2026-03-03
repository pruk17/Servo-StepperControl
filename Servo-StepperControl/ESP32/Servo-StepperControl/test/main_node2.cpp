#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

#include "Ultra_detector.h" 
#include "CtrlManager.h"
//ros2 topic pub --once /plant_command std_msgs/msg/String "{data: '1'}" 
// ตัวแปรสำหรับกลไก
PlantingManager robot;

// micro-ROS entities
rcl_publisher_t ultra_pub;
rcl_publisher_t feedback_pub;
rcl_subscription_t command_sub;
std_msgs__msg__Float32 ultra_msg;
std_msgs__msg__Int32 feedback_msg;
std_msgs__msg__String command_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum ros_state { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
ros_state state = WAITING_AGENT;

// Callback เมื่อได้รับคำสั่งจาก Topic "plant_command"
void command_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  String cmd = String(msg->data.data);
  
  if (cmd == "1") robot.startPlantPattern();
  else if (cmd == "0") robot.LoadPattern();
  else if (cmd == "2") robot.testpattern();
  else if (cmd == "s") robot.stopAll();
}

void setup() {
  Serial.begin(115200);
  delay(2000);                 // IMPORTANT for ESP32 USB
  set_microros_serial_transports(Serial);
  delay(2000);                 // allow transport to open

  rmw_uros_set_custom_transport(
    true,
    (void *) &Serial,
    platformio_transport_open,
    platformio_transport_close,
    platformio_transport_write,
    platformio_transport_read
  );
  setupUltra();// ตั้งค่า Sensor
  robot.begin();

  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 69);

  Serial.println("Waiting for agent...");
  while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
    Serial.println("Agent not ready");
    delay(1000);
  }
  Serial.println("Agent connected");

  // เตรียมพื้นที่หน่วยความจำสำหรับรับ String
  command_msg.data.data = (char * ) malloc(20 * sizeof(char));
  command_msg.data.capacity = 20;
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, "node2_ctrl", "", &support);

  // 1. Publisher: ระยะทาง Ultrasonic
  rclc_publisher_init_default(&ultra_pub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "ultra_distance");

  // 2. Publisher: Feedback ขั้นตอนการทำงาน (Step 1, 2, 3...)
  rclc_publisher_init_default(&feedback_pub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "plant_feedback");

  // 3. Subscriber: รับคำสั่ง 1, 0, s
  rclc_subscription_init_default(&command_sub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "plant_command");

  // Executor สำหรับ 1 Subscriber
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &command_sub, &command_msg, &command_callback, ON_NEW_DATA);

}

void loop() {
  robot.update();

  switch(state) {
    case WAITING_AGENT:
      if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
      break;
    case AGENT_AVAILABLE:
      state = AGENT_CONNECTED;
      break;
    case AGENT_CONNECTED:
      if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { state = AGENT_DISCONNECTED; break; }

      // ส่งค่า Ultrasonic
      ultra_msg.data = getStableDistance();
      rcl_publish(&ultra_pub, &ultra_msg, NULL);

      // ส่งค่า Feedback (Step ปัจจุบันของ Robot)
      feedback_msg.data = robot.getCurrentStep(); // **ต้องเพิ่มฟังก์ชันนี้ใน CtrlManager
      rcl_publish(&feedback_pub, &feedback_msg, NULL);

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;
    case AGENT_DISCONNECTED:
      ESP.restart();
      break;
  }
  delay(10);
}