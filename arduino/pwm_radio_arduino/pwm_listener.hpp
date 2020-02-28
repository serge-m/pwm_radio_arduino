class PwmListener
{
  volatile unsigned int pwm_value_;
  volatile unsigned long prev_time_;
  int default_value_;
  unsigned long inactivity_th_;
public:
  const int pin;
  const int pin_as_pc_int;
 

public:
  PwmListener(int pin_, int default_value, unsigned long inactivity_th):
    pin(pin_), pwm_value_(default_value), prev_time_(0), pin_as_pc_int(digitalPinToPCINT(pin_)),
  default_value_(default_value), inactivity_th_(inactivity_th)
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
 
  bool is_active() {
    return micros_since_last_signal < inactivity_th;
  }
  unsigned int value() const {
    if (is_active()) 
      return pwm_value_;
    else
      return default_value_;      
  }

  unsigned long prev_time_micros() const {
    return prev_time_;
  }
};
