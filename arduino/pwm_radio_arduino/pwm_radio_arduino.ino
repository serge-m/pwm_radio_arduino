#include <PinChangeInterrupt.h>
#include <timer.h>
#include <ros.h>
#include "ros_lib/ackermann_msgs/AckermannDrive.h"
#include "range_transforms.hpp"
#include "pca9685_driver.hpp"

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

constexpr int DEFAULT_PULSE_WIDTH = 1400;
volatile unsigned long last_update_micros_by_serial = 0;
ros::NodeHandle nh;



Range3Levels<float> angle_input(-1, 0, 1);
Range3Levels<float> angle_pwm(1000, 1400, 1800);
Range3Levels<float> speed_input(-1, 0, 1);
Range3Levels<float> speed_pwm(1000, 1400, 1800);
volatile float pwm_steering_by_serial = 0;
volatile float pwm_throttle_by_serial = 0;


PwmController pwm_controller(PCA9685_FREQUENCY, 10);

Timer<3> timer;

void handle_driver_pwm(const ackermann_msgs::AckermannDrive& msg) {
  if (all_params_are_there) { 
    pwm_steering_by_serial = msg.steering_angle; //transform(msg.steering_angle, angle_input, angle_pwm);
    pwm_throttle_by_serial = msg.speed; //transform(msg.speed, speed_input, speed_pwm);
  }
}

ackermann_msgs::AckermannDrive radio_pwm_message;
ros::Subscriber<ackermann_msgs::AckermannDrive> subscriber_driver_pwm("pwm_radio_arduino/driver_pwm", &handle_driver_pwm);
ros::Publisher publisher_radio("pwm_radio_arduino/radio_ackermann", &radio_pwm_message);



bool check_pwm_out_enabled() {
  return digitalRead(pin_pwm_out_enabled) == HIGH;
}

bool drive_according_to_input(void *)
{
  unsigned int steering = 0;
  unsigned int throttle = 0;

//  if (mode == drive_by_radio) {
//    digitalWrite(LED_BUILTIN, HIGH);
//  }
//  else {
//    digitalWrite(LED_BUILTIN, LOW);
//  }
  // TODO: set according to the mode
  steering = pwm_steering_by_serial;
  throttle = pwm_throttle_by_serial;
  
  if (check_pwm_out_enabled()) {
    pwm_controller.set(channel_pwm_out_throttle, throttle);
    pwm_controller.set(channel_pwm_out_steering, steering);
  }
  else {
    pwm_controller.reset();
  }

  
  return true;
}

void publish_state_to_ros() {
  radio_pwm_message.steering_angle = pwm_steering_by_serial; // TODO: replace with real values
  radio_pwm_message.speed = pwm_throttle_by_serial;          // TODO: replace with real values
  publisher_radio.publish(&radio_pwm_message);
}


bool ros_publish_and_spin(void *) {
  publish_state_to_ros();
  nh.spinOnce();
  return true;
}

bool reload_parameters(void *) {
  int zz;
  if (!nh.getParam("/pwm_radio_arduino/mode", &zz, 1, 0)) { 
//     digitalWrite(LED_BUILTIN, HIGH);
zz = 17;  
    
  }
  else {
//    digitalWrite(LED_BUILTIN, LOW);
    
  }
  if (zz > 0) { 
     digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  pwm_steering_by_serial = zz;
  return true; 
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  // preparing timers
  timer.every(20, drive_according_to_input);
  timer.every(20, ros_publish_and_spin);
  timer.every(501, reload_parameters);

  // init ros
  nh.initNode();
  nh.advertise(publisher_radio);
  nh.subscribe(subscriber_driver_pwm);
}



void loop() { 
  timer.tick();
}
