#include <PinChangeInterrupt.h>


namespace pwm_listener {
constexpr int pin_in_angle = 7;
constexpr int pin_in_speed = 8;
const int pcint_angle = digitalPinToPCINT(pin_in_angle);
const int pcint_speed = digitalPinToPCINT(pin_in_speed);


class PwmListener
{
  volatile unsigned int pwm_value_;
  volatile unsigned long prev_time_;
public:
  int pin_as_pc_int;

public:
  PwmListener(int pin_as_pc_int, int default_value):
    pwm_value_(default_value), prev_time_(0), pin_as_pc_int(pin_as_pc_int)
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

PwmListener angle_listener(pcint_angle, DEFAULT_PULSE_WIDTH);
PwmListener speed_listener(pcint_speed, DEFAULT_PULSE_WIDTH);
void interrupt_steering() { angle_listener.process(); }
void interrupt_throttle() { speed_listener.process(); }

void pwm_listener_setup() {
  pinMode(pin_in_angle, INPUT);
  pinMode(pin_in_speed, INPUT);
  attachPinChangeInterrupt(angle_listener.pin_as_pc_int, &interrupt_steering, CHANGE);
  attachPinChangeInterrupt(speed_listener.pin_as_pc_int, &interrupt_throttle, CHANGE);
}
} // namespace
