#include <Adafruit_PWMServoDriver.h>


class PwmController
{
  bool init_ = false;
  unsigned int frequency_;
  Adafruit_PWMServoDriver pwm;

public:
  PwmController(unsigned int frequency, unsigned int tolerance):
    frequency_(frequency)
  {
    // called this way, it uses the default address 0x40
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
  }

  void set(unsigned int channel, int value) {
    if (!init_) {
      initialize();
    }
    pwm.setPWM(channel, 0, pulseWidth(value));
  }

  void initialize() {
    pwm.begin();
    pwm.setPWMFreq(frequency_);
    init_ = true;
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