// odometry_kaia.h — nguyên bản từ code của bạn (chỉ tính, đã tắt String__assign
// vì chu kỳ khác nhau giữa các bản micro_ros). TPR/WHEEL_BASE đã là số thật.
#pragma once
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <nav_msgs/msg/odometry.h>
#include "config.h"

extern volatile long g_leftCount, g_rightCount;
extern long prevOdomL, prevOdomR;
extern float odomX, odomY, odomYaw;
extern rcl_publisher_t odom_pub;
extern nav_msgs__msg__Odometry odom_msg;

inline void odometryPoll(float dt) {
  if (dt < 0.01f) return;
  long cl, cr;
  noInterrupts();
  cl = g_leftCount; cr = g_rightCount;
  interrupts();
  long dL = cl - prevOdomL, dR = cr - prevOdomR;
  prevOdomL = cl; prevOdomR = cr;

  float distL = (dL / ENCODER_TPR) * WHEEL_CIRCUMFERENCE;
  float distR = (dR / ENCODER_TPR) * WHEEL_CIRCUMFERENCE;
  float distC = (distL + distR) / 2.0f;
  float dYaw = (distR - distL) / WHEEL_BASE;

  float yawMid = odomYaw + dYaw / 2.0f;
  odomX += distC * cos(yawMid);
  odomY += distC * sin(yawMid);
  odomYaw += dYaw;
  while (odomYaw > M_PI) odomYaw -= 2.0f * M_PI;
  while (odomYaw < -M_PI) odomYaw += 2.0f * M_PI;

  memset(&odom_msg, 0, sizeof(odom_msg));
  odom_msg.pose.pose.position.x = odomX;
  odom_msg.pose.pose.position.y = odomY;
  odom_msg.pose.pose.orientation.z = sin(odomYaw / 2.0f);
  odom_msg.pose.pose.orientation.w = cos(odomYaw / 2.0f);
  odom_msg.twist.twist.linear.x = distC / dt;
  odom_msg.twist.twist.angular.z = dYaw / dt;
  rcl_publish(&odom_pub, &odom_msg, NULL);
}
