#include <Adafruit_PWMServoDriver.h>

// pin 9 is connected to the power V+ of pca9685
// to capture when the controller is enabled to reinitialize it.
constexpr int pin_pwm_out_enabled = 9;

constexpr int channel_angle = 1;
constexpr int channel_speed = 0;
float pwm_angle_zero = 0;
float pwm_speed_zero = 0;


bool esc_init = false;

constexpr int PCA9685_FREQUENCY = 60;


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


bool check_pwm_out_enabled() {
  return digitalRead(pin_pwm_out_enabled) == HIGH;
}

void pwm_init() {
  
}

int pulseWidth(int pulse_width)
{
  return (int)(float(pulse_width) / 1000000 * PCA9685_FREQUENCY * 4096);
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
    pwm.begin();
    pwm.setPWMFreq(PCA9685_FREQUENCY);
//    pwm.setPWM(channel_angle, 0, 0);
//    pwm.setPWM(channel_speed, 0, 0);
    delay(1000);  // TODO: check if we need this 
    esc_init = true;
    return 3;
  }

  pwm.setPWM(channel_angle, 0, pulseWidth(pwm_angle));
  pwm.setPWM(channel_speed, 0, pulseWidth(pwm_speed));

  return 0;
}

    
