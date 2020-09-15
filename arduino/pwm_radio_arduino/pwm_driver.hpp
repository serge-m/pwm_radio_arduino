#include <Servo.h>

namespace pwm_driver {
// pin 9 is connected to the power V+ of ESC
// to capture when it is enabled to reinitialize it.
constexpr int pin_esc_on = 9;

constexpr int pin_angle = 10;
constexpr int pin_speed = 11;

constexpr int angle_zero_usec = 1500;
constexpr int speed_zero_usec = 1500;

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
    servo_angle.writeMicroseconds(angle_zero_usec);
    servo_speed.writeMicroseconds(speed_zero_usec);
    delay(2000);  // TODO: check if we need this 
    esc_init = true;
}    

int pwm_spin(int angle_usec, int speed_usec) {
  if(!esc_on()) {
    esc_init = false;
    servo_angle.writeMicroseconds(angle_zero_usec);
    servo_speed.writeMicroseconds(speed_zero_usec);
    return 1;
  }
    
  if (!esc_init) {
    init_zero_level_on_esc();
    return 3;
  }

  servo_angle.writeMicroseconds(angle_usec);
  servo_speed.writeMicroseconds(speed_usec);

  return 0;
}
}
