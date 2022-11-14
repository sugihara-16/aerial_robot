#include <IcsHardSerialClass.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <SPI.h>


const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 1000;
const int open_pos = 8200;
const int close_pos = 11300;

IcsHardSerialClass krs(&Serial,EN_PIN,BAUDRATE,TIMEOUT);

ros::NodeHandle nh;

void messageCb(const std_msgs::String& msg) {
  krs.begin();
  String command = msg.data;
  if(command == "open"){
    krs.setPos(0,open_pos); //open the hand
    nh.loginfo("Open the hand");
  }else{
    krs.setPos(0,close_pos); //close the hand
    nh.loginfo("Close the hand");
  }
   nh.getHardware()->setBaud(BAUDRATE); 
}


ros::Subscriber<std_msgs::String> hand_command_sub("hand_command", &messageCb);

void setup() {
  krs.begin();
  krs.setPos(0,close_pos); //close the hand
  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();
  nh.subscribe(hand_command_sub);
}


void loop() {
  nh.spinOnce();
  delay(30);
}
