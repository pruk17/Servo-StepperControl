#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#include "Ultra_detector.h" 
//docker run -it --rm -v /dev:/dev --privileged --net=host -e ROS_DOMAIN_ID=69 microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200

// ตัวแปรสำหรับ micro-ROS
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

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

  /* ---------- support ---------- */
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  /* ---------- node ---------- */
  rclc_node_init_default(&node, "ultra_node", "", &support);
  /* ---------- publisher ---------- */
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "ultra_distance");
  /* ---------- executor ---------- */
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  msg.data = 0.0;
}

enum ros_state {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

ros_state state = WAITING_AGENT;

void loop() {

  switch(state) {

    case WAITING_AGENT:
      if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK) {
        state = AGENT_AVAILABLE;
      }
      break;

    case AGENT_AVAILABLE:
      // reinitialize micro-ROS here if needed
      state = AGENT_CONNECTED;
      break;

    case AGENT_CONNECTED:
      if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        state = AGENT_DISCONNECTED;
        break;
      }

      msg.data = getStableDistance();
      rcl_publish(&publisher, &msg, NULL);
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;

    case AGENT_DISCONNECTED:
      // reset micro-ROS session
      Serial.println("Agent lost. Resetting ROS...");
      ESP.restart();   // simplest reliable solution on ESP32
      break;
  }

  delay(50);
}