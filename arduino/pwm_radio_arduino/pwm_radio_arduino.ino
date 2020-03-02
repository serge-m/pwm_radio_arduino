#include <PinChangeInterrupt.h>
#include <timer.h>
#include <ros.h>
#include "ros_lib/ackermann_msgs/AckermannDrive.h"
#include "ros_lib/pwm_radio_arduino/steering_tfm.h"
#include "pca9685_driver.hpp"
#include "range_transforms.hpp"

#define PCA9685_FREQUENCY             60

enum drive_mode {
  drive_off = 0,
  drive_by_radio = 1,
  drive_by_serial = 2,
  drive_by_serial_with_radio_limit = 3
};

int mode = drive_by_radio;
bool all_params_are_there = true;


// pin 9 is connected to the power V+ of pca9685
// to capture when the controller is enabled to reinitialize it.
const int pin_pwm_out_enabled = 9;

const int channel_pwm_out_steering = 1;
const int channel_pwm_out_throttle = 0;

volatile unsigned long last_update_micros_by_serial = 0;
ros::NodeHandle nh;


pwm_radio_arduino::steering_tfm tfm_ranges;
ackermann_msgs::AckermannDrive ackermanm_ros_in;
ackermann_msgs::AckermannDrive ackermann_ros_out;
volatile float pwm_steering_by_serial = 0;
volatile float pwm_throttle_by_serial = 0;


PwmController pwm_controller(PCA9685_FREQUENCY, 10);

Timer<3> timer;

void update() {
  if (all_params_are_there) { 
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
  update();
}


void ros_callback_transform_range(const pwm_radio_arduino::steering_tfm& msg) {
  tfm_ranges = msg;
  update();
}


ros::Subscriber<ackermann_msgs::AckermannDrive> sub_driver_input("pwm_radio_arduino/driver_pwm", &ros_callback_driver_input);
ros::Subscriber<pwm_radio_arduino::steering_tfm> sub_tfm_range("pwm_radio_arduino/steering_tfm", &ros_callback_transform_range);
ros::Publisher publisher_radio("pwm_radio_arduino/radio_ackermann", &ackermann_ros_out);

bool check_pwm_out_enabled() {
  return digitalRead(pin_pwm_out_enabled) == HIGH;
}

bool drive_according_to_input(void *)
{
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
