#include <PinChangeInterrupt.h>
#include <ros.h>
#include "ros_lib/ackermann_msgs/AckermannDrive.h"
#include "ros_lib/pwm_radio_arduino/steering_tfm.h"
#include "pca9685_driver.hpp"
#include "range_transforms.hpp"

constexpr unsigned long INACTIVITY_TH_MICROS = 1000000;
constexpr int DEFAULT_SPEED_OFF = 0;


volatile unsigned long last_update_micros = 0;
ros::NodeHandle nh;

bool tfm_init = false;
pwm_radio_arduino::steering_tfm tfm_ranges;
ackermann_msgs::AckermannDrive ackermann_ros_in;
ackermann_msgs::AckermannDrive ackermann_ros_out;


void update(
  const float& angle_in, 
  const float& speed_in, 
  const pwm_radio_arduino::steering_tfm& tfm,
  float& angle_out, 
  float& speed_out
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
  last_update_micros = micros();
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
//  
//  update(
//    ackermann_ros_in.steering_angle, ackermann_ros_in.speed, 
//    tfm_ranges, 
//    ackermann_ros_out.steering_angle, ackermann_ros_out.speed
//  );
  update(
    tfm_ranges.angle_out_zero, tfm_ranges.speed_out_zero, 
    tfm_ranges, 
    pwm_angle_zero, pwm_speed_zero
  );
}


ros::Subscriber<ackermann_msgs::AckermannDrive> sub_driver_input("pwm_radio_arduino/driver_ackermann", &ros_callback_driver_input);
ros::Subscriber<pwm_radio_arduino::steering_tfm> sub_tfm_range("pwm_radio_arduino/steering_tfm", &ros_callback_transform_range);
ros::Publisher publisher_radio("pwm_radio_arduino/arduino_ackermann", &ackermann_ros_out);


bool drive_according_to_input(void *)
{
  int res = 0;
  if (!tfm_init) {
    ackermann_ros_out.speed = 1400;
    res += 10;
  }
  else if (micros() - last_update_micros > INACTIVITY_TH_MICROS) {
    ackermann_ros_out.speed = 1400;
    res += 20;
  }
  else {
    res += 30;
  }
  
  res += pwm_spin(ackermann_ros_out.steering_angle, ackermann_ros_out.speed);
  ackermann_ros_out.jerk = res;

  publisher_radio.publish(&ackermann_ros_out);
  
  return true;
}


//bool ros_publish_and_spin(void *) {
//  
//  
//  return true;
//}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // init ros
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(publisher_radio);
  nh.subscribe(sub_driver_input);
  nh.subscribe(sub_tfm_range);
}



void loop() { 
  for ( int i =0; i < 30; ++i ) {
    nh.spinOnce();
    delay(2);
  }
  drive_according_to_input(nullptr);

  delay(2);
}
