// micro_ros_kaia_fun.h — transport Kaia 2 tham số + sync giờ host.
// CẢNH BÁO: bản chính thức dùng 4 tham số (ssid,pass,host,port); gọi 4 tham số
// trên fork Kaia = Guru Meditation PC=0x00000000. Ở đây WiFi nối tay bằng
// WiFi.begin(), transport chỉ nhận (AGENT_IP, AGENT_PORT).
#pragma once
#include <Arduino.h>
#include <micro_ros_kaia.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <kaiaai_msgs/msg/kaiaai_telemetry2.h>
#include <nav_msgs/msg/odometry.h>
#include <rmw_microros/rmw_microros.h>
#include "config.h"
#include "motor_control.h"

extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_subscription_t leftRPM_sub, rightRPM_sub;
extern std_msgs__msg__Float32 leftRPM_msg, rightRPM_msg;
extern rcl_publisher_t telem_pub, odom_pub;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { Serial.print("RCL ERROR: "); Serial.println(rc); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

inline void leftRPMCallback(const void *msgin) {
  onLeftTarget(((const std_msgs__msg__Float32 *)msgin)->data);
}
inline void rightRPMCallback(const void *msgin) {
  onRightTarget(((const std_msgs__msg__Float32 *)msgin)->data);
}

inline bool microRosInit() {
  Serial.print("Connecting to agent ");
  Serial.print(AGENT_IP);
  Serial.print(" ... ");
  set_microros_wifi_transports(AGENT_IP, AGENT_PORT);  // 2 THAM SỐ — fork Kaia
  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_kaia", "", &support));
  RCCHECK(rclc_subscription_init_default(&leftRPM_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/motor/left_target_rpm"));
  RCCHECK(rclc_subscription_init_default(&rightRPM_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/motor/right_target_rpm"));
  if (rclc_publisher_init_best_effort(&telem_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(kaiaai_msgs, msg, KaiaaiTelemetry2),
      "/telemetry") != RCL_RET_OK) {
    Serial.println("telemetry publisher that bai");
    return false;
  }
  if (rclc_publisher_init_default(&odom_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "/wheel/odom") != RCL_RET_OK) {
    Serial.println("/wheel/odom publisher that bai");
    return false;
  }
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &leftRPM_sub, &leftRPM_msg,
    &leftRPMCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &rightRPM_sub, &rightRPM_msg,
    &rightRPMCallback, ON_NEW_DATA));
  if (rmw_uros_sync_session(1000) != RCL_RET_OK) {
    Serial.println("sync that bai");
  } else {
    Serial.print("OK, dong ho = ");
    Serial.println((long)(rmw_uros_epoch_millis() / 1000));
  }
  Serial.println("success");
  return true;
}
