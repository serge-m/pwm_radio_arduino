#include <Servo.h>

// pin 9 is connected to the power V+ of ESC
// to capture when it is enabled to reinitialize it.
constexpr int pin_esc_on = 9;

constexpr int pin_angle = 10;
constexpr int pin_speed = 11;

constexpr int pwm_angle_zero = 90;
constexpr int pwm_speed_zero = 90;

Servo servo_angle;
Servo servo_speed;
bool esc_init = false;  // after (re-)start of ESC zero speed level has to be set

void servo_setup() { // call in setup()
  servo_angle.attach(pin_angle);
  servo_speed.attach(pin_speed);
}

bool esc_on() {
  return digitalRead(pin_esc_on) == HIGH;
}

void init_zero_level_on_esc() {
    servo_angle.write(pwm_angle_zero);
    servo_speed.write(pwm_speed_zero);
    delay(2000);  // TODO: check if we need this 
    esc_init = true;
}    

int pwm_spin(const int pwm_angle, const int pwm_speed) {
  if(!esc_on()) {
    esc_init = false;
    servo_angle.write(pwm_angle_zero);
    servo_speed.write(pwm_speed_zero);
    return 1;
  }
    
  if (!esc_init) {
    init_zero_level_on_esc();
    return 3;
  }

  servo_angle.write(pwm_angle);
  servo_speed.write(pwm_speed);

  return 0;
}
