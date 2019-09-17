#include <PinChangeInterrupt.h>
#include <Adafruit_PWMServoDriver.h>
#include <timer.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include "ros_lib/pwm_radio_arduino/pwm_steering.h"


#define DEFAULT_PULSE_WIDTH   1400
#define FREQUENCY             60
#define DEFAULT_THROTTLE_FROM_RADIO 1500
#define MIN_IN_THROTTLE_PULSE_WIDTH 1000
#define MAX_IN_THROTTLE_PULSE_WIDTH 2000
#define MICROS_RADIO_INACTIVITY_THRESHOLD 500000

#define MIN_OUT_THROTTLE_PULSE_WIDTH (DEFAULT_PULSE_WIDTH-230)
#define MAX_OUT_THROTTLE_PULSE_WIDTH (DEFAULT_PULSE_WIDTH+110)

#define CLAMP(x, upper, lower) (min(upper, max(x, lower)))

enum drive_mode {
  drive_off = 0,
  drive_by_radio = 1,
  drive_by_serial = 2
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
volatile unsigned long last_update_micros_by_serial = 0;
ros::NodeHandle nh;

pwm_radio_arduino::pwm_steering radio_pwm_message;

void handle_driver_pwm(const pwm_radio_arduino::pwm_steering& driver_pwm_message) {
  val_steering_by_serial = driver_pwm_message.steering;
  val_throttle_by_serial = driver_pwm_message.throttle;
  last_update_micros_by_serial = micros();
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

  void process() {
    uint8_t trigger = getPinChangeInterruptTrigger(pin_as_pc_int);
    if(trigger == RISING)
      prev_time_ = micros();
    else if(trigger == FALLING)
      pwm_value_ = micros_since_last_signal();
    else {
      // Wrong usage
    }
  }

  unsigned long micros_since_last_signal() const {
    return micros() - prev_time_; 
  }

  unsigned int value() const {
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

unsigned interp(unsigned value, unsigned in_min, unsigned in_max, unsigned out_min, unsigned out_max) {
  return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min);
}

unsigned limit_throttle_pwm(unsigned throttle) {
  if (throttle >= DEFAULT_THROTTLE_FROM_RADIO) {
    return interp(throttle, DEFAULT_THROTTLE_FROM_RADIO, MAX_IN_THROTTLE_PULSE_WIDTH, DEFAULT_PULSE_WIDTH, MAX_OUT_THROTTLE_PULSE_WIDTH);
  }
  else {
    return interp(throttle, MIN_IN_THROTTLE_PULSE_WIDTH, DEFAULT_THROTTLE_FROM_RADIO, MIN_OUT_THROTTLE_PULSE_WIDTH, DEFAULT_PULSE_WIDTH);
  }
}

bool drive_according_to_input(void *)
{
  unsigned int steering = DEFAULT_PULSE_WIDTH;
  unsigned int throttle = DEFAULT_PULSE_WIDTH;

  
  if (mode == drive_by_radio) { 
    if (pwm_listener_steering.micros_since_last_signal() < MICROS_RADIO_INACTIVITY_THRESHOLD) {
      steering =   pwm_listener_steering.value();
      throttle =   limit_throttle_pwm(pwm_listener_throttle.value());
    }
  }
  else if (mode == drive_by_serial) {
    if (micros() - last_update_micros_by_serial < MICROS_RADIO_INACTIVITY_THRESHOLD) {
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
