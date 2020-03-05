/////////////////////////////////////////////////
// Demonstration of the ESC initialization using 
// PCA9685. 
// Arduino is connected to PCA9685 with I2C
// PCA9685 is connected to ESC using 3 pin connector.
// Depending on the moment you switch on the
// ESC you have different speed and direction.
//
// https://serge-m.github.io/  2020
/////////////////////////////////////////////////


#include <Adafruit_PWMServoDriver.h>


// using the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Analog servos run at ~60 Hz updates
int frequency_ = 60;


void setup() {
  Serial.begin(57600);
  Serial.println("Starting servo test");

  pwm.begin();
  pwm.setPWMFreq(frequency_);


  pinMode(LED_BUILTIN, OUTPUT);
  delay(10);

  
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Initialization of ESC");
  set_and_delay(0, 1400, 1000);
  digitalWrite(LED_BUILTIN, LOW);
}

int pulseWidth(int pulse_width) // width between 0 and 4096
{
  return (int)(float(pulse_width) / 1000000 * frequency_ * 4096);
}

void set_and_delay(int servonum, int pulse_width, int delay_time) {
  Serial.print("Running on servo ");
  Serial.print(servonum);
  Serial.print(" pulse ");
  Serial.println(pulse_width);

  pwm.setPWM(servonum, 0, pulseWidth(pulse_width));
  delay(delay_time);
}


void loop() {
  set_and_delay(0, 1480, 500);
  set_and_delay(0, 1280, 500);
}
