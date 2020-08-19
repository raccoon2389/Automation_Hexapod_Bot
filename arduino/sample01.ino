#include <ros.h>
#include <std_msgs/String.h>


ros::NodeHandle nh;

String dd;

void messageCb(const std_msgs::String &msg)
{
  dd = msg.data;
  if(dd == "w"){
    digitalWrite(52, HIGH);
  }
  else{
    digitalWrite(52, LOW);
  }
}

ros::Subscriber<std_msgs::String> sub("ww", &messageCb);

void setup()
{
  pinMode(52, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}