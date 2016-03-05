#include <ros/ros.h>
#include <vector>
#include <phidgets_interface_kit/AnalogArray.h>

using namespace std;

vector<ros::Publisher> rangePubs;
ros::Subscriber analogInSub;

int main(int argc, char** argv) {
  ros::init(argc, argv, "phidgets_ir_range");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  //digitalOutSub = nh.subscribe("cmd_digital_out", 1, onCmdDigitalOut);
}
