#include <ros.h>
#include "ros_lib/pwm_radio_arduino/control.h"
#include "pwm_driver.hpp"
#include "pwm_listener.hpp"
constexpr unsigned long MICROS_RADIO_INACTIVITY_THRESHOLD=100000;
constexpr unsigned long DRIVER_INACTIVITY_TH_MILLIS = 300;

constexpr int led_status = 12;
volatile unsigned long last_update_millis = 0;
ros::NodeHandle nh;

pwm_radio_arduino::control control_driver_in;
pwm_radio_arduino::control control_out;

void ros_callback_driver_input(const pwm_radio_arduino::control& msg) {
  last_update_millis = millis();
  control_driver_in = msg;
}

ros::Subscriber<pwm_radio_arduino::control> sub_driver_input("pwm_radio_arduino/control_driver", &ros_callback_driver_input);
ros::Publisher publisher("pwm_radio_arduino/control_result", &control_out);


bool radio_is_up_to_date() {
  // Function uses angle IN channel to determine if the radio is on.
  // Speed channel is sends default (1500usec) value after the radio is switched off. Therefore speed channel cannot be used for this purpose.
  return pwm_listener::angle_listener.micros_since_last_signal() < MICROS_RADIO_INACTIVITY_THRESHOLD;
}


bool driver_is_up_to_date() {
  return millis() - last_update_millis < DRIVER_INACTIVITY_TH_MILLIS;
}

bool radio_is_static() {
  return (
    abs(static_cast<int>(pwm_listener::speed_listener.value()) - pwm_driver::speed_zero_usec) < 50 and
    abs(static_cast<int>(pwm_listener::angle_listener.value()) - pwm_driver::angle_zero_usec) < 50
    );    
}

int safe_speed(int speed) {
  // original speed controls are way too fast. I reduce the range here so that the car drives slower.
  return (speed - pwm_driver::speed_zero_usec) / 2 + pwm_driver::speed_zero_usec;
}

bool drive_according_to_input(void *)
{
  if (!radio_is_up_to_date()) {
    control_out.speed_control_usec = pwm_driver::speed_zero_usec;
    control_out.angle_control_usec = pwm_driver::angle_zero_usec;
    digitalWrite(led_status, HIGH * millis() / 64 % 2);   // fast blinking if no radio is acquired
  } else if (driver_is_up_to_date() && radio_is_static()) {
    control_out = control_driver_in;
    digitalWrite(led_status, HIGH * millis() / 256 % 2);  // medium blinking if driving by driver
  } else {  // drive according to radio
    control_out.angle_control_usec = pwm_listener::angle_listener.value();
    control_out.speed_control_usec = safe_speed(pwm_listener::speed_listener.value());
    digitalWrite(led_status, HIGH * millis() / 1024 % 2);  // slow blinking if driving by radio
  }
  
  pwm_driver::pwm_spin(control_out.angle_control_usec, control_out.speed_control_usec);
  publisher.publish(&control_out);
  return true;
}

void setup() {
  pwm_listener::pwm_listener_setup();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(led_status, OUTPUT);
  pwm_driver::servo_setup();
  
  // init ros
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(publisher);
  nh.subscribe(sub_driver_input);
}



void loop() { 
  nh.spinOnce();
  if ((millis() * 17) % 7 == 0) { // send updates not too often 
    drive_according_to_input(nullptr);
  }
  delay(5);
}
