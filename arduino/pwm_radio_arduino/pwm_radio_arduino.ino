#include <PinChangeInterrupt.h>
#include <timer.h>
#include <ros.h>
#include "ros_lib/ackermann_msgs/AckermannDrive.h"
#include "ros_lib/pwm_radio_arduino/pwm_steering.h"
#include "range_transforms.hpp"

enum drive_mode {
  drive_off = 0,
  drive_by_radio = 1,
  drive_by_serial = 2,
  drive_by_serial_with_radio_limit = 3
};

volatile drive_mode mode = drive_by_radio;
bool all_params_are_there = false;


// pin 9 is connected to the power V+ of pca9685
// to capture when the controller is enabled to reinitialize it.
const int pin_pwm_out_enabled = 9;

const int channel_pwm_out_steering = 1;
const int channel_pwm_out_throttle = 0;

constexpr int DEFAULT_PULSE_WIDTH = 1400;
volatile int val_steering_by_serial = DEFAULT_PULSE_WIDTH;
volatile int val_throttle_by_serial = DEFAULT_PULSE_WIDTH;
volatile unsigned long last_update_micros_by_serial = 0;
ros::NodeHandle nh;



Range3Levels<float> angle_input(0, 0, 0);
Range3Levels<float> angle_pwm(0, 0, 0);
Range3Levels<float> speed_input(0, 0, 0);
Range3Levels<float> speed_pwm(0, 0, 0);


void handle_driver_pwm(const ackermann_msgs::AckermannDrive& msg) {
  if (all_params_are_there) { 
    pwm_steering_by_serial = transform(msg.rotation, angle_input, angle_pwm);
    pwm_throttle_by_serial = transform(msg.speed, speed_input, speed_pwm);
  }
}

ros::Subscriber<ackermann_msgs::AckermannDrive> subscriber_driver_pwm("pwm_radio_arduino/driver_pwm", &handle_driver_pwm);
ros::Publisher publisher_radio("pwm_radio_arduino/radio_pwm", &radio_pwm_message);


bool drive_according_to_input(void *)
{
  unsigned int steering = 0;
  unsigned int throttle = 0;
  
  if (mode == drive_by_radio) { 
      steering =   pwm_listener_steering.value();
      throttle =   pwm_listener_throttle.value();
  }
  else if (mode == drive_by_serial_with_radio_limit) {
    if (
      micros() - last_update_micros_by_serial < MICROS_RADIO_INACTIVITY_THRESHOLD && 
      pwm_listener_steering.is_active()
    ) {
      steering = val_steering_by_serial;
      throttle = val_throttle_by_serial;
    }
  }

  if (check_pwm_out_enabled()) {
    pwm_controller.set(channel_pwm_out_throttle, throttle);
    pwm_controller.set(channel_pwm_out_steering, steering);
  }
  else {
    pwm_controller.reset();
  }
  return true;
}


bool handle_ros(void *) {
  publish_state_to_ros(
);
  nh.spinOnce();
  return true;
}

bool reload_parameters(void *) {
  if (!nh.getParam("~mode", mode)) { 
        mode = drive_by_radio;
  }
}

void setup() {
  // preparing timers
  timer.every(20, drive_according_to_input);
  timer.every(20, handle_ros);
  timer.every(2000, reload_parameters);

  // init ros
  nh.initNode();
  nh.advertise(publisher_radio);
  nh.subscribe(subscriber_mode);
  nh.subscribe(subscriber_driver_pwm);
}



void loop() { 
  timer.tick();
}
