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
float sensorFoV;

float rangeValueMin = 0.0f;
float rangeValueMax = 0.0f;

vector<ros::Publisher> rangePubs;
ros::Subscriber analogInSub;

void onAnalogIn(const phidgets_interface_kit::AnalogArrayConstPtr& input){

  ROS_INFO("onAnalogIn: sensor count=%i", sensor_indexes.size());

  for(int i=0; i < sensor_indexes.size(); i++){
      int sensorIndex = sensor_indexes[i];

      sensor_msgs::Range range;
      range.header.stamp = ros::Time::now();
      range.header.frame_id = frames[i];
      range.radiation_type = sensor_msgs::Range::INFRARED;
      range.min_range = rangeValueMin;
      range.max_range = rangeValueMax;
      range.field_of_view = sensorFoV;

      range.range = (analogValueScale/(input->values[sensorIndex] + analogValueOffset))/100.0f;

      if(range.range > range.max_range){
        range.range = range.max_range;
      }
      else if(range.range < range.min_range){
        range.range = range.min_range;
      }

      rangePubs[i].publish(range);
  }
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
  nh.getParam("sensor_fov", sensorFoV);

  rangeValueMin = (analogValueScale/(analogValueMin + analogValueOffset))/100.0f;
  rangeValueMax = (analogValueScale/(analogValueMax + analogValueOffset))/100.0f;

  analogInSub = n.subscribe(topic.c_str(), 1, onAnalogIn);

  vector<int>::iterator it=sensor_indexes.begin();
  for (; it != sensor_indexes.end(); ++it) {
    rangePubs.push_back(nh.advertise<sensor_msgs::Range>(frames[*it].c_str(), 1, false));
  }

  ros::spin();
}
