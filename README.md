# pwm_radio_arduino
Node for pwm control of robocar via pwm_radio_arduino.control

## Architecture
The car consists  of the following parts:
* computer running ROS
* arduino that listens for PWM commands  and sends them to PCA9865 controller via I2C.
  It uses rosserial module from ROS for serial communication and ROS messages handling.
  The commands can come either
  * from the computer (connected via USB)
  * or the radio receiver (connected via pins 9 and 10)

  The mode of operation is defined by rostopic `pwm_radio_arduino/mode`.

  Output of the radio receiver is constantly redirected to `pwm_radio_arduino/radio_pwm`.
* PCA9865
* radio RC module
* ESC


## Development

### How to add/update message headers for arduino sketch

Activate catkin environment. 

    rosrun rosserial_client make_libraries arduino/pwm_radio_arduino
