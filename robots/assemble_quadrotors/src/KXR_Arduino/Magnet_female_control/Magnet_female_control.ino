#include <IcsHardSerialClass.h>
#include <IcsSoftSerialClass.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <SPI.h>

const byte S_RX_PIN = 4;
const byte S_TX_PIN = 3;
const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 200;
const int open_pos = 10000;
const int close_pos = 4000;

IcsSoftSerialClass krs(S_RX_PIN,S_TX_PIN,EN_PIN,BAUDRATE,TIMEOUT);

ros::NodeHandle nh;

void messageCb(const std_msgs::String& msg) {
  krs.begin();
  String command = msg.data;
  if(command == "open"){
    krs.setSpd(0,127);
    krs.setPos(0,open_pos); //open the hand
    nh.loginfo("Magnet activated");
  }else{
    krs.setSpd(0,60);
    krs.setPos(0,close_pos); //close the hand
    nh.loginfo("Magnet inactivated");
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
