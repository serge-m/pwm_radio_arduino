#include <PinChangeInterrupt.h>
#include <timer.h>
#include <ros.h>
#include "ros_lib/ackermann_msgs/AckermannDrive.h"
#include "ros_lib/pwm_radio_arduino/steering_tfm.h"
#include "pca9685_driver.hpp"
#include "range_transforms.hpp"

#define PCA9685_FREQUENCY             60

constexpr int channel_pwm_out_steering = 1;
constexpr int channel_pwm_out_throttle = 0;
constexpr unsigned long INACTIVITY_TH_MICROS = 300000;
constexpr int DEFAULT_SPEED_OFF = 0;


volatile unsigned long last_update_micros = 0;
ros::NodeHandle nh;

bool tfm_init = false;
pwm_radio_arduino::steering_tfm tfm_ranges;
ackermann_msgs::AckermannDrive ackermanm_ros_in;
ackermann_msgs::AckermannDrive ackermann_ros_out;


PwmController pwm_controller(PCA9685_FREQUENCY);

Timer<2> timer;

void update() {
  if (tfm_init) { 
    ackermann_ros_out.steering_angle = transform2(
      ackermanm_ros_in.steering_angle, 
      tfm_ranges.angle_in_low, tfm_ranges.angle_in_zero, tfm_ranges.angle_in_high,
      tfm_ranges.angle_out_low, tfm_ranges.angle_out_zero, tfm_ranges.angle_out_high
      );
    ackermann_ros_out.speed = transform2(
      ackermanm_ros_in.speed, 
      tfm_ranges.speed_in_low, tfm_ranges.speed_in_zero, tfm_ranges.speed_in_high,
      tfm_ranges.speed_out_low, tfm_ranges.speed_out_zero, tfm_ranges.speed_out_high
      );
  }
}

void ros_callback_driver_input(const ackermann_msgs::AckermannDrive& msg) {
  ackermanm_ros_in = msg;
  last_update_micros = micros();
  ackermann_ros_out.jerk = last_update_micros / 1000000.;
  update();
}


void ros_callback_transform_range(const pwm_radio_arduino::steering_tfm& msg) {
  tfm_ranges = msg;
  tfm_init = true;
  update();
}


ros::Subscriber<ackermann_msgs::AckermannDrive> sub_driver_input("pwm_radio_arduino/driver_ackermann", &ros_callback_driver_input);
ros::Subscriber<pwm_radio_arduino::steering_tfm> sub_tfm_range("pwm_radio_arduino/steering_tfm", &ros_callback_transform_range);
ros::Publisher publisher_radio("pwm_radio_arduino/arduino_ackermann", &ackermann_ros_out);


bool drive_according_to_input(void *)
{
  if (!tfm_init || micros() - last_update_micros > INACTIVITY_TH_MICROS) {
    ackermann_ros_out.speed = DEFAULT_SPEED_OFF;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  
  if (check_pwm_out_enabled()) {
    pwm_controller.set(channel_pwm_out_throttle, ackermann_ros_out.speed);
    pwm_controller.set(channel_pwm_out_steering, ackermann_ros_out.steering_angle);
  }
  else {
    pwm_controller.reset();
  }
  
  return true;
}


bool ros_publish_and_spin(void *) {
  publisher_radio.publish(&ackermann_ros_out);
  nh.spinOnce();
  return true;
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // preparing timers
  timer.every(20, drive_according_to_input);
  timer.every(20, ros_publish_and_spin);

  // init ros
  nh.initNode();
  nh.advertise(publisher_radio);
  nh.subscribe(sub_driver_input);
  nh.subscribe(sub_tfm_range);
}



void loop() { 
  timer.tick();
}
