#include <PinChangeInterrupt.h>
#include <ros.h>
#include "ros_lib/ackermann_msgs/AckermannDrive.h"
#include "ros_lib/pwm_radio_arduino/conversion_settings.h"
#include "pwm_driver.hpp"
#include "range_transforms.hpp"
#include "pwm_listener.hpp"

constexpr unsigned long INACTIVITY_TH_MILLIS = 500;
constexpr int led_status = 12;
volatile unsigned long last_update_millis = 0;
ros::NodeHandle nh;

bool settings_init = false;
pwm_radio_arduino::conversion_settings settings;
ackermann_msgs::AckermannDrive ackermann_ros_in;
ackermann_msgs::AckermannDrive ackermann_ros_out;

template<typename T> void update(
  const float& angle_in, 
  const float& speed_in, 
  const pwm_radio_arduino::control_range& input_range,
  const pwm_radio_arduino::control_range& output_range,
  T& angle_out, 
  T& speed_out
  ) {
    angle_out = transform2(
      angle_in, 
      input_range.angle.low, input_range.angle.zero, input_range.angle.high,
      output_range.angle.low, output_range.angle.zero, output_range.angle.high
      );
    speed_out = transform2(
      speed_in, 
      input_range.speed.low, input_range.speed.zero, input_range.speed.high,
      output_range.speed.low, output_range.speed.zero, output_range.speed.high
      );
}

void ros_callback_driver_input(const ackermann_msgs::AckermannDrive& msg) {
  ackermann_ros_in = msg;
  last_update_millis = millis();
  if (settings_init) {
    update(
      ackermann_ros_in.steering_angle, ackermann_ros_in.speed, 
      settings.driver_in,
      settings.out, 
      ackermann_ros_out.steering_angle, ackermann_ros_out.speed
    );
  }
}


void ros_callback_settings(const pwm_radio_arduino::conversion_settings& msg) {
  settings = msg;
  settings_init = true;
}


ros::Subscriber<ackermann_msgs::AckermannDrive> sub_driver_input("pwm_radio_arduino/driver_ackermann", &ros_callback_driver_input);
// Using ros messages instead of dymparams for the settings because dynparams are not supported. Normal ros params are not dynamic enough
ros::Subscriber<pwm_radio_arduino::conversion_settings> sub_tfm_range{
  "pwm_radio_arduino/settings", &ros_callback_settings
};
ros::Publisher publisher_radio("pwm_radio_arduino/arduino_ackermann", &ackermann_ros_out);


bool drive_according_to_input(void *)
{
  int res = 0;
  if (!settings_init) {
    digitalWrite(led_status, HIGH);
    return;
  }

 
  if (millis() - last_update_millis > INACTIVITY_TH_MILLIS) {
    ackermann_ros_out.speed = pwm_speed_zero;
    digitalWrite(led_status, HIGH * millis() / 64 % 2);   // fast blinking if no command from ROS 
  }
  else {
    digitalWrite(led_status, HIGH * millis() / 256 % 2);  // slow blinking if everythin is ok
  }
  
  pwm_spin(ackermann_ros_out.steering_angle, ackermann_ros_out.speed);

  ackermann_ros_out.steering_angle_velocity = pwm_listener_steering.value();
  ackermann_ros_out.acceleration = pwm_listener_throttle.value();
  publisher_radio.publish(&ackermann_ros_out);
  
  return true;
}

void setup() {
  pwm_listener_setup();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(led_status, OUTPUT);
  servo_setup();
  
  // init ros
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(publisher_radio);
  nh.subscribe(sub_driver_input);
  nh.subscribe(sub_tfm_range);
}



void loop() { 
  nh.spinOnce();
  if ((millis() * 17) % 7 == 0) { // send updates not too often 
    drive_according_to_input(nullptr);
  }
  delay(5);
}
