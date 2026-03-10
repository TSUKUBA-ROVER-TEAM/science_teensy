#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Adafruit_ADS1X15.h>
#include <Wire.h>

#include <rcl/logging.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float64.h>

Adafruit_ADS1115 ads;

const float R_REF = 1000.0;
const float R0 = 100.0;
const float ALPHA = 0.003851;

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t temperature_publisher;
std_msgs__msg__Float64 temperature_msg;

long last_time = 0;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

// エラーハンドリングループ
void error_loop() {
  while (1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    long now = millis();
    if (now - last_time >= 10) {
      int16_t raw_a0 = ads.readADC_SingleEnded(0);
      float volt_a0 = ads.computeVolts(raw_a0);

      float v_supply = 3.30;
      float current = (v_supply - volt_a0) / R_REF;

      float r_pt100 = volt_a0 / current;

      float temperature = (r_pt100 / R0 - 1.0) / ALPHA;

      temperature_msg.data = temperature;

      RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));

      last_time = now;
    }
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &temperature_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "temperature"));

  // create timer for 20ms update rate
  const unsigned int timer_timeout = 100; // 100 Hz
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_US_TO_NS(timer_timeout),
                                  timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Wire.begin();

  if (!ads.begin()) {
    while (1)
      ;
  }

  ads.setGain(GAIN_FOUR);
  ads.setDataRate(RATE_ADS1115_16SPS);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}