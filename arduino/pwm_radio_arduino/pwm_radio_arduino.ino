#include <PinChangeInterrupt.h>
#include <Adafruit_PWMServoDriver.h>
#include <timer.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include "ros_lib/pwm_radio_arduino/pwm_steering.h"

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1380
#define FREQUENCY             60

enum drive_mode {
  drive_off = 0,
  drive_by_radio = 1,
  drive_by_serial= 2
};

volatile drive_mode mode = drive_by_radio;

const int pin_pwm_in_steering = 11;
const int pin_pwm_in_throttle = 10;

// pin 9 is connected to the power V+ of pca9685 
// to capture when the controller is enabled to reinitialize it.
const int pin_pwm_out_enabled = 9;

const int channel_pwm_out_steering = 1;
const int channel_pwm_out_throttle = 0;

volatile int val_steering_by_serial = DEFAULT_PULSE_WIDTH;
volatile int val_throttle_by_serial = DEFAULT_PULSE_WIDTH;

ros::NodeHandle nh;

pwm_radio_arduino::pwm_steering radio_pwm_message;

void handle_driver_pwm(const pwm_radio_arduino::pwm_steering& driver_pwm_message) {
  val_steering_by_serial = driver_pwm_message.steering;
  val_throttle_by_serial = driver_pwm_message.throttle;
}

void handle_mode_change(const std_msgs::Int32& msg) {
  mode = static_cast<drive_mode>(msg.data);
}


ros::Subscriber<std_msgs::Int32> subscriber_mode("pwm_radio_arduino/mode", &handle_mode_change);
ros::Subscriber<pwm_radio_arduino::pwm_steering> subscriber_driver_pwm("pwm_radio_arduino/driver_pwm", &handle_driver_pwm);
ros::Publisher publisher_radio("pwm_radio_arduino/radio_pwm", &radio_pwm_message);


class PwmController
{
  static const unsigned int max_num_channels = 16;
  bool init_ = false;
  const unsigned int tolerance_;
  unsigned int old_values[max_num_channels] = {0};
  unsigned int frequency_;
  Adafruit_PWMServoDriver pwm;
  
public:
  PwmController(unsigned int frequency, unsigned int tolerance): 
    frequency_(frequency), tolerance_(tolerance)  
  {
    // called this way, it uses the default address 0x40
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
  }

  bool set(unsigned int channel, int value) {
    if (!init_) {
      initialize();
    }
    if (abs(int(value) - int(old_values[channel])) > tolerance_) {
      pwm.setPWM(channel, 0, pulseWidth(value));  
      old_values[channel] = value;
      return true;
    }
    return false;
  }

  void initialize() {
//    Serial.println("info: waiting for output pwm controller..."); 
    for (auto i = 0; i < max_num_channels; ++i) {
      old_values[i] = 0;
    }
    pwm.begin();
    pwm.setPWMFreq(frequency_);
    init_ = true;
//    Serial.println("info: output pwm controller initialized"); 
  }

  void reset() {
    init_ = false;
  }    
 
private:
  int pulseWidth(int pulse_width)
  {
    return (int)(float(pulse_width) / 1000000 * frequency_ * 4096);
  }
};


class PwmListener 
{
  volatile unsigned int pwm_value_;
  volatile unsigned long prev_time_;
public:
  const int pin;
  const int pin_as_pc_int;

public:
  PwmListener(int pin_, int default_value): 
    pin(pin_), pwm_value_(default_value), prev_time_(0), pin_as_pc_int(digitalPinToPCINT(pin_))
  {}

  void process()
  {
    uint8_t trigger = getPinChangeInterruptTrigger(pin_as_pc_int);
    if(trigger == RISING)
      prev_time_ = micros();
    else if(trigger == FALLING)
      pwm_value_ = micros() - prev_time_;
    else {
      // Wrong usage
    }
  }
  
  unsigned int value() const 
  {
    return pwm_value_;
  }

  unsigned long prev_time_micros() const {
    return prev_time_;
  }
};


PwmListener pwm_listener_steering (pin_pwm_in_steering, DEFAULT_PULSE_WIDTH);
PwmListener pwm_listener_throttle (pin_pwm_in_throttle, DEFAULT_PULSE_WIDTH);
void interrupt_steering() { pwm_listener_steering.process(); }
void interrupt_throttle() { pwm_listener_throttle.process(); }

PwmController pwm_controller(FREQUENCY, 10);
Timer<2> timer;

bool check_pwm_out_enabled() {
  auto val = digitalRead(pin_pwm_out_enabled);
  return val == HIGH;
}



bool drive_according_to_input(void *) 
{
  int steering = 0;
  int throttle = 0;

  if (mode == drive_by_radio) {   
    steering =   pwm_listener_steering.value();
    throttle =   pwm_listener_throttle.value();
    throttle = throttle - 1500 + DEFAULT_PULSE_WIDTH;
  }
  else if (mode == drive_by_serial) {
    steering = val_steering_by_serial;
    throttle = val_throttle_by_serial;
  } 
  else if (mode == drive_off) {
    steering = DEFAULT_PULSE_WIDTH;
    throttle = DEFAULT_PULSE_WIDTH;
    return true;
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

void setup() {
  // starting input pwm monitoring...
  pinMode(pwm_listener_steering.pin, INPUT);
  attachPinChangeInterrupt(pwm_listener_steering.pin_as_pc_int, &interrupt_steering, CHANGE);
  pinMode(pwm_listener_throttle.pin, INPUT);
  attachPinChangeInterrupt(pwm_listener_throttle.pin_as_pc_int, &interrupt_throttle, CHANGE);

  // preparing timers
  timer.every(10, drive_according_to_input);

  // init ros
  nh.initNode();
  nh.advertise(publisher_radio);
  nh.subscribe(subscriber_mode);
  nh.subscribe(subscriber_driver_pwm);
}

void publish_state_to_ros() {
  radio_pwm_message.throttle = pwm_listener_throttle.value();
  radio_pwm_message.steering = pwm_listener_steering.value();
  publisher_radio.publish(&radio_pwm_message);
}


void loop() { 
  timer.tick();
  publish_state_to_ros();
  nh.spinOnce();
  //delay(100);
}
