#include <Servo.h>

Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position

constexpr int led_blink = 12;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(led_blink, OUTPUT);
  myservo.attach(10);  // attach the servo on pin to the servo object

}

void loop() {
  int start = 90-30; // degree start
  int end = 90+30;   // degree end
  digitalWrite(LED_BUILTIN, HIGH);
  
  for (pos = start; pos <= end; pos += 1) { 
    myservo.write(pos);              
    digitalWrite(led_blink, !digitalRead(led_blink));
    delay(100);                       
  }

  digitalWrite(LED_BUILTIN, LOW);
  
  for (pos = end; pos >= start; pos -= 1) { 
    myservo.write(pos);              
    digitalWrite(led_blink, !digitalRead(led_blink));
    delay(100);                      
  }
}
