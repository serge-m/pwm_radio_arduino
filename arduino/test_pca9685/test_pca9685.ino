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

void blink1(int times, int duration_blink, int duration_after=1) {
  for (int i = 0; i < times; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(duration_blink/2);
    digitalWrite(LED_BUILTIN, LOW);
    delay(duration_blink/2);
  }
  delay(duration_after);
}

void loop() {
  set_and_delay(0, 1400, 1000);
  blink1(3, 100);
  set_and_delay(0, 1600, 1000);
  blink1(2, 100);
  set_and_delay(0, 1200, 1000);
  blink1(1, 100);
  set_and_delay(0, 1500, 1000);
}
