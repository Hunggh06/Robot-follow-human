// kaia_esp32.ino — main gọn kiểu Leonardo (setup/loop), ruột là code của bạn.
// LIDAR: nguyên bản (lidar_kaia.h). ĐỘNG CƠ: đã refactor sang motor_control.*
// nhưng giữ TPR 234 + luật dấu ISR + gains của bạn.
#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <math.h>

#include "config.h"
#include "encoder.h"
#include "motor_control.h"
#include "lidar_kaia.h"
#include "odometry_kaia.h"
#include "micro_ros_kaia_fun.h"

// ---- Globals (khai báo TRƯỚC mọi hàm dùng — quy tắc của bạn) ----
volatile long g_leftCount = 0;
volatile long g_rightCount = 0;
long prevOdomL = 0, prevOdomR = 0;
unsigned long lastOdomMs = 0;
float odomX = 0.0f, odomY = 0.0f, odomYaw = 0.0f;

uint8_t ldsBuf[LDS_BUF_MAX];
uint32_t telemSeq = 0;

HardwareSerial LIDAR(2);

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t leftRPM_sub, rightRPM_sub;
std_msgs__msg__Float32 leftRPM_msg, rightRPM_msg;
rcl_publisher_t telem_pub, odom_pub;
kaiaai_msgs__msg__KaiaaiTelemetry2 telem_msg;
nav_msgs__msg__Odometry odom_msg;

// Encoder objects kiểu Leonardo, luật dấu của bạn:
// trái A==B ++ ; phải A==B -- (invert=true)
KaiaEncoder encL(ENC_L_A, ENC_L_B, false);
KaiaEncoder encR(ENC_R_A, ENC_R_B, true);

void IRAM_ATTR leftISR() {
  // Giữ đúng ISR gốc của bạn (đọc cả A/B, không qua class để tránh overhead):
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);
  if (A == B) g_leftCount++;
  else g_leftCount--;
}
void IRAM_ATTR rightISR() {
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);
  if (A == B) g_rightCount--;
  else g_rightCount++;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== KAIA LEONARDO ESP32 ===");

  motorInit();
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);
  lidarInit();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi ");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.print("IP ");
  Serial.println(WiFi.localIP());

  microRosInit();
  lastOdomMs = millis();
  Serial.println("Robot ready.");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  lidarPoll();    // CẤM SỬA LOGIC — nguyên bản Kaia
  motorUpdate();  // PID 10Hz đã refactor
  unsigned long nowOdom = millis();
  if (nowOdom - lastOdomMs >= 100) {
    float dtOdom = (nowOdom - lastOdomMs) / 1000.0f;
    lastOdomMs = nowOdom;
    if (dtOdom > 0.001f && dtOdom < 1.0f) odometryPoll(dtOdom);
  }
}
