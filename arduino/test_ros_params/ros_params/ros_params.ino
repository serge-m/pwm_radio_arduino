#include <ros.h>

#include <std_msgs/Int32.h>

constexpr int led_status = 12;
//create your handle
ros::NodeHandle nh;
std_msgs::Int32 output;
ros::Publisher publisher("stat", &output);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(led_status, OUTPUT);

  nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.advertise(publisher);
}

int value = 50;

void loop() {
  
  digitalWrite(led_status, HIGH);
  delay(value);
  digitalWrite(led_status, LOW);
  delay(value);
  
  nh.spinOnce();
  digitalWrite(led_status, HIGH);
  delay(value);
  digitalWrite(led_status, LOW);
  delay(value);

  nh.spinOnce();
  
  output.data = value;
  publisher.publish(&output);

  nh.spinOnce();
  
  if (!nh.getParam("param", &value) ){ 
    value = 177;        
  }
}
