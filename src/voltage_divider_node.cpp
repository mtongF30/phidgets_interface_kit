#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <phidgets_interface_kit/AnalogArray.h>


using namespace std;

//! Parameters
int sensorIndex;
float analogValueOffset;
float analogValueScale;

ros::Publisher voltagePub;
ros::Subscriber analogInSub;

void onAnalogIn(const phidgets_interface_kit::AnalogArrayConstPtr& input) {

  if(sensorIndex < input->values.size()){
    std_msgs::Float32 value;

    value.data = ((float)input->values[sensorIndex] * analogValueScale) + analogValueOffset;
    voltagePub.publish(value);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "voltage_divider_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  string topic;

  nh.getParam("topic", topic);
  nh.getParam("sensor_index", sensorIndex);
  nh.getParam("value_offset", analogValueOffset);
  nh.getParam("value_scale", analogValueScale);

  ROS_INFO("analogValueOffset = %f", analogValueOffset);
  ROS_INFO("analogValueScale = %f", analogValueScale);

  analogInSub = n.subscribe(topic.c_str(), 1, onAnalogIn);
  voltagePub = nh.advertise<std_msgs::Float32>("voltage", 10);

  ros::spin();

  return 0;
}
