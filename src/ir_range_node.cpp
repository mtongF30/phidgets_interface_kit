#include <ros/ros.h>
#include <map>
#include <vector>
#include <phidgets_interface_kit/AnalogArray.h>
#include <sensor_msgs/Range.h>

using namespace std;

//! Parameters
vector<int> sensor_indexes;
vector<string> frames;
int analogValueOffset;
int analogValueScale;
int analogValueMin;
int analogValueMax;

vector<ros::Publisher> rangePubs;
ros::Subscriber analogInSub;

void onAnalogIn(const phidgets_interface_kit::AnalogArrayConstPtr& input){
  //
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ir_range_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  string topic;

  nh.getParam("topic", topic);
  nh.getParam("sensor_indexes", sensor_indexes);
  nh.getParam("frames", frames);
  nh.getParam("value_offset", analogValueOffset);
  nh.getParam("value_scale", analogValueScale);
  nh.getParam("value_min", analogValueMin);
  nh.getParam("value_max", analogValueMax);

  ROS_INFO("Sensors: %i", sensor_indexes.size());
  ROS_INFO("Topic: %s", topic.c_str());

  analogInSub = nh.subscribe(topic.c_str(), 1, onAnalogIn);

  vector<int>::iterator it=sensor_indexes.begin();
  for (; it != sensor_indexes.end(); ++it) {

    ros::Publisher pub = nh.advertise<sensor_msgs::Range>(frames[*it].c_str(), 1, false);
    rangePubs.push_back(pub);
  }

  ros::spin();
}
