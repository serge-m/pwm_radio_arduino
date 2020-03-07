#include <Adafruit_PWMServoDriver.h>

// pin 9 is connected to the power V+ of pca9685
// to capture when the controller is enabled to reinitialize it.
constexpr int pin_pwm_out_enabled = 9;

constexpr int channel_angle = 1;
constexpr int channel_speed = 0;
int pwm_angle_zero = 0;
int pwm_speed_zero = 0;


bool esc_init = false;

constexpr int PCA9685_FREQUENCY = 60;


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm_drv = Adafruit_PWMServoDriver();


bool check_pwm_out_enabled() {
  return digitalRead(pin_pwm_out_enabled) == HIGH;
}

int pulseWidth(int pulse_width)
{
  return pulse_width * PCA9685_FREQUENCY * 4096 / 1000000;
}

void set_pwm_(int channel, int pwm) {
  pwm_drv.setPWM(channel, 0, pulseWidth(pwm));
}

void init_pwm_() {
  pwm_drv.begin();
  pwm_drv.setPWMFreq(PCA9685_FREQUENCY);
}

int pwm_spin(const int pwm_angle, const int pwm_speed) {
  if(!check_pwm_out_enabled()) {
    esc_init = false;
    return 1;
  }
    
  if (!esc_init) {
    if(pwm_speed_zero==0 && pwm_angle_zero==0) {  // zero values not initialized, do nothing
      return 2;
    }
    init_pwm_();
//    set_pwm_(channel_angle, 0);
//    set_pwm_(channel_speed, pwm_speed_zero);
//    delay(3000);  // TODO: check if we need this 
    esc_init = true;
    return 3;
  }

//  set_pwm_(channel_angle, pwm_angle);
//  set_pwm_(channel_speed, pwm_speed);

  return 0;
}

    
