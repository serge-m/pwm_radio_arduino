#include <PinChangeInterrupt.h>
#include <ros.h>
#include "ros_lib/ackermann_msgs/AckermannDrive.h"
#include "ros_lib/pwm_radio_arduino/steering_tfm.h"
#include "pwm_driver.hpp"
#include "range_transforms.hpp"

constexpr unsigned long INACTIVITY_TH_MILLIS = 500;
constexpr int led_status = 12;
volatile unsigned long last_update_millis = 0;
ros::NodeHandle nh;

bool tfm_init = false;
pwm_radio_arduino::steering_tfm tfm_ranges;
ackermann_msgs::AckermannDrive ackermann_ros_in;
ackermann_msgs::AckermannDrive ackermann_ros_out;
int pwm_speed = 0;
int pwm_angle = 0;

template<typename T> void update(
  const float& angle_in, 
  const float& speed_in, 
  const pwm_radio_arduino::steering_tfm& tfm,
  T& angle_out, 
  T& speed_out
  ) {
    angle_out = transform2(
      angle_in, 
      tfm.angle_in_low, tfm.angle_in_zero, tfm.angle_in_high,
      tfm.angle_out_low, tfm.angle_out_zero, tfm.angle_out_high
      );
    speed_out = transform2(
      speed_in, 
      tfm.speed_in_low, tfm.speed_in_zero, tfm.speed_in_high,
      tfm.speed_out_low, tfm.speed_out_zero, tfm.speed_out_high
      );
}

void ros_callback_driver_input(const ackermann_msgs::AckermannDrive& msg) {
  ackermann_ros_in = msg;
  last_update_millis = millis();
  if (tfm_init) {
    update(
      ackermann_ros_in.steering_angle, ackermann_ros_in.speed, 
      tfm_ranges, 
      ackermann_ros_out.steering_angle, ackermann_ros_out.speed
    );
  }
}


void ros_callback_transform_range(const pwm_radio_arduino::steering_tfm& msg) {
  tfm_ranges = msg;
  tfm_init = true;
}


ros::Subscriber<ackermann_msgs::AckermannDrive> sub_driver_input("pwm_radio_arduino/driver_ackermann", &ros_callback_driver_input);
ros::Subscriber<pwm_radio_arduino::steering_tfm> sub_tfm_range("pwm_radio_arduino/steering_tfm", &ros_callback_transform_range);
ros::Publisher publisher_radio("pwm_radio_arduino/arduino_ackermann", &ackermann_ros_out);


bool drive_according_to_input(void *)
{
  int res = 0;
  if (!tfm_init) {
    digitalWrite(led_status, HIGH);
    return;
  }

 
  if (millis() - last_update_millis > INACTIVITY_TH_MILLIS) {
    ackermann_ros_out.speed = pwm_speed_zero;
    digitalWrite(led_status, HIGH * millis() / 64 % 2);    
  }
  else {
    digitalWrite(led_status, HIGH * millis() / 256 % 2);
  }
  
  pwm_spin(ackermann_ros_out.steering_angle, ackermann_ros_out.speed);

  publisher_radio.publish(&ackermann_ros_out);
  
  return true;
}

void setup() {
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
