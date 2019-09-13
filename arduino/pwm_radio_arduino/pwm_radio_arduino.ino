#include <PinChangeInterrupt.h>
#include <Adafruit_PWMServoDriver.h>
#include <timer.h>
#include <ros.h>
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

char buf[50] = "";

volatile int val_steering_by_serial = DEFAULT_PULSE_WIDTH;
volatile int val_throttle_by_serial = DEFAULT_PULSE_WIDTH;

ros::NodeHandle nh;

pwm_radio_arduino::pwm_steering ros_pwm_message;
ros::Publisher chatter("pwm_radio_arduino", &ros_pwm_message);

class PwmController
{
  static const unsigned int max_num_channels = 16;
  bool init_ = false;
  const unsigned int tolerance_;
  unsigned int old_values[max_num_channels] = {0};
  unsigned int frequency_;
  Adafruit_PWMServoDriver pwm;

  int pulseWidth(int pulse_width)
  {
    return (int)(float(pulse_width) / 1000000 * frequency_ * 4096);
  }


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
};


class PwmListener 
{
  volatile unsigned int pwm_value_;
  volatile unsigned long prev_time_;
  const int pin_;

public:
  PwmListener(int pin, int default_value): 
    pin_(pin), pwm_value_(default_value), prev_time_(0)
  {}

  void process()
  {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(pin_));
    if(trigger == RISING)
      prev_time_ = micros();
    else if(trigger == FALLING)
      pwm_value_ = micros() - prev_time_;
    else {
      // Wrong usage
    }
  }

  // wrapper function is required as an argument  
  // because attachPinChangeInterrupt doesn't accept 
  // pointers to object's methods
  void setup(void(*func)()) 
  {
    pinMode(pin_, INPUT);
    attachPinChangeInterrupt(digitalPinToPCINT(pin_), func, CHANGE);
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

void interrupt1() 
{
  pwm_listener_steering.process();
}

PwmListener pwm_listener_throttle (pin_pwm_in_throttle, DEFAULT_PULSE_WIDTH);
void interrupt2() 
{
  pwm_listener_throttle.process();
}

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
    bool changed = false;
    changed |= pwm_controller.set(channel_pwm_out_throttle, throttle);
    changed |= pwm_controller.set(channel_pwm_out_steering, steering);
  
    if (changed)
    {
      sprintf(buf, "set: %d %d\n", steering, throttle);
//      Serial.print(buf);
    }
  }
  else {
    pwm_controller.reset();
  }
  return true;
}

bool process_input(void *) {
//  sCmd.readSerial(); 
  return true;  
}
//
//void command() {
//  const char *arg = sCmd.next();
//  
//  if (arg != NULL) {
//     mode = static_cast<drive_mode>(atoi(arg));
//  }
//
//  if (mode == drive_by_serial) {
//    const char *arg_steering = sCmd.next();
//    const char *arg_throttle = sCmd.next();
//    if (arg_steering != NULL and arg_throttle != NULL) {
//      val_steering_by_serial = atoi(arg_steering);
//      val_throttle_by_serial = atoi(arg_throttle);
//    }
//    else {
//      //Serial.print("ret: error=missing_argument");
//      return;
//    }
//  }
//  
//  //Serial.print("ret: mode=");
//  //Serial.println(mode);
//}

void read_radio() {
  static char b[30];
  sprintf(b, "ret: s=%d t=%d\n", pwm_listener_steering.value(), pwm_listener_throttle.value());
  //Serial.print(b);  
}

void setup() {
  //Serial.begin(115200);
  //Serial.println("info: starting input pwm monitoring...");
  pwm_listener_steering.setup(&interrupt1);
  pwm_listener_throttle.setup(&interrupt2);

  //Serial.println("info: preparing timers");
  timer.every(10, drive_according_to_input);
  timer.every(11, process_input);

  nh.initNode();
  nh.advertise(chatter);

  //Serial.println("info: preparing command processing");
  
  //Serial.println("info: setup complete");
}



void loop() { 
//   timer.tick();
  ros_pwm_message.throttle = pwm_listener_throttle.value();
  ros_pwm_message.steering = pwm_listener_steering.value();
  chatter.publish(&ros_pwm_message);
  nh.spinOnce();
//  delay(100);
  
}
