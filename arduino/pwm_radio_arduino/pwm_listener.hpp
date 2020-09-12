#include <PinChangeInterrupt.h>

const int pin_pwm_in_steering = 7;
const int pin_pwm_in_throttle = 8;

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

void pwm_listener_setup() {
  pinMode(pwm_listener_steering.pin, INPUT);
  attachPinChangeInterrupt(pwm_listener_steering.pin_as_pc_int, &interrupt_steering, CHANGE);
  pinMode(pwm_listener_throttle.pin, INPUT);
  attachPinChangeInterrupt(pwm_listener_throttle.pin_as_pc_int, &interrupt_throttle, CHANGE);
}
