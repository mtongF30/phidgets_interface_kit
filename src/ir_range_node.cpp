#include <ros/ros.h>
#include <map>
#include <vector>
#include <limits>
#include <phidgets_interface_kit/AnalogArray.h>
#include <sensor_msgs/Range.h>
#include <angles/angles.h>

using namespace std;

//! Parameters
vector<int> sensor_indexes;
vector<string> frames;
float analogValueOffset;
float analogValueScale;
float analogValueMin;
float analogValueMax;
float sensorFoV;

float rangeValueMin = 0.0f;
float rangeValueMax = 0.0f;

ros::Publisher rangePub;
ros::Subscriber analogInSub;
phidgets_interface_kit::AnalogArray analogValues;

void onAnalogIn(const phidgets_interface_kit::AnalogArrayConstPtr& input) {

  for(int i=0; i < sensor_indexes.size(); i++){
    int sensorIndex = sensor_indexes[i];

    sensor_msgs::Range range;
    range.header.stamp = ros::Time::now();
    range.header.frame_id = frames[i];
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.min_range = rangeValueMin;
    range.max_range = rangeValueMax;
    range.field_of_view = angles::from_degrees(sensorFoV);

    range.range = (analogValueScale/(input->values[sensorIndex] + analogValueOffset))/100.0f;

    if(range.range > range.max_range){
      range.range = INFINITY;
    }
    else if(range.range < range.min_range){
      range.range = INFINITY;
    }

    rangePub.publish(range);
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

  ROS_INFO("analogValueOffset = %f", analogValueOffset);
  ROS_INFO("analogValueScale = %f", analogValueScale);
  ROS_INFO("analogValueMin = %f", analogValueMin);
  ROS_INFO("analogValueMax = %f", analogValueMax);
  ROS_INFO("sensorFoV = %f", sensorFoV);

  rangeValueMax = (analogValueScale/(analogValueMin + analogValueOffset))/100.0f;
  rangeValueMin = (analogValueScale/(analogValueMax + analogValueOffset))/100.0f;

  analogInSub = n.subscribe(topic.c_str(), 1, onAnalogIn);
  rangePub = nh.advertise<sensor_msgs::Range>("range", 10);

  ros::spin();

  return 0;
}
