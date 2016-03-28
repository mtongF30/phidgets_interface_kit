/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets motor control HC
 *  Copyright (c) 2010, Bob Mottram
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <phidgets_api/phidget.h>
#include <phidgets_interface_kit/AnalogArray.h>
#include <phidgets_interface_kit/DigitalArray.h>
#include "interface_kit.h"

ros::Publisher analogInPub;
ros::Publisher digitalInPub;
ros::Publisher digitalOutPub;
ros::Subscriber digitalOutSub;

CPhidgetInterfaceKitHandle ifkit = 0;
bool initialised = false;


int AttachHandler(CPhidgetHandle phid, void *userptr){
  int serial_number;
  const char *name;

  CPhidget_getDeviceName (phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d attached!", name, serial_number);

  return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr){
  int serial_number;
  const char *name;

  CPhidget_getDeviceName (phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d detached!", name, serial_number);

  return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *Description){
    ROS_ERROR("Error handled. %d - %s", ErrorCode, Description);
    return 0;
}

int display_properties(CPhidgetInterfaceKitHandle phid){
	int serialNo, version, numInputs, numOutputs, numSensors, triggerVal, ratiometric;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetInterfaceKit_getInputCount(phid, &numInputs);
	CPhidgetInterfaceKit_getOutputCount(phid, &numOutputs);
	CPhidgetInterfaceKit_getSensorCount(phid, &numSensors);
	CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);

	ROS_INFO("%s\n", ptr);
	ROS_INFO("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	ROS_INFO("# Digital Inputs: %d\n# Digital Outputs: %d\n", numInputs, numOutputs);
	ROS_INFO("# Sensors: %d\n", numSensors);
	ROS_INFO("Ratiometric: %d\n", ratiometric);

	for(int i = 0; i < numSensors; i++)
	{
    int maxRate = -1;
    int minRate = -1;
    int rate = -1;
		CPhidgetInterfaceKit_getSensorChangeTrigger (phid, i, &triggerVal);
    CPhidgetInterfaceKit_getDataRate(phid, i, &rate);
    CPhidgetInterfaceKit_getDataRateMax(phid, i, &maxRate);
    CPhidgetInterfaceKit_getDataRateMin(phid, i, &minRate);

		ROS_INFO("Sensor#: %d, Trigger=%d, Max=%d, Min=%d, Rate=%d\n", i, triggerVal, maxRate, minRate, rate);
	}

	return 0;
}

bool attach( CPhidgetInterfaceKitHandle &phid, int serial_number){

  //create the InterfaceKit object
	CPhidgetInterfaceKit_create(&phid);

	//Setup life-cycle handlers
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)phid, ErrorHandler, NULL);

  // Open
	CPhidget_open((CPhidgetHandle)phid, serial_number);

  return true;
}


void disconnect(CPhidgetInterfaceKitHandle &phid){
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

void publishAnalogInputStates(CPhidgetInterfaceKitHandle &phid) {
  int count = 0;
  phidgets_interface_kit::AnalogArray analogArray;
  CPhidgetInterfaceKit_getSensorCount(phid, &count);

  for(int i=0; i<count; i++){
    int value = 0;
    CPhidgetInterfaceKit_getSensorValue(phid, i, &value);

    analogArray.values.push_back(value);
  }

  analogInPub.publish(analogArray);
}

void publishDigitalInputStates(CPhidgetInterfaceKitHandle &phid) {
  int count = 0;
  phidgets_interface_kit::DigitalArray digitalArray;
  CPhidgetInterfaceKit_getInputCount(phid, &count);

  for(int i=0; i<count; i++){
    int value = 0;
    CPhidgetInterfaceKit_getInputState(phid, i, &value);

    digitalArray.states.push_back((bool)value);
  }

  digitalInPub.publish(digitalArray);
}

void publishDigitalOutputStates(CPhidgetInterfaceKitHandle &phid) {
  int count = 0;
  phidgets_interface_kit::DigitalArray digitalArray;
  CPhidgetInterfaceKit_getOutputCount(phid, &count);

  for(int i=0; i<count; i++){
    int value = 0;
    CPhidgetInterfaceKit_getOutputState(phid, i, &value);

    digitalArray.states.push_back((bool)value);
  }

  digitalOutPub.publish(digitalArray);
}

void onCmdDigitalOut(const phidgets_interface_kit::DigitalArrayConstPtr& input){

  if(initialised){
    int count = 0;
    CPhidgetInterfaceKit_getOutputCount(ifkit, &count);

    int limit = (input->states.size() < count) ? input->states.size() : count;

    for(int i=0; i<limit; i++){
        int value = input->states[i];
        CPhidgetInterfaceKit_setOutputState(ifkit, i, value);
    }
  }
  else{
    ROS_WARN("Phidgets Interface Kit not initalized yet, ignoring output command");
  }
}

void timerCallback(const ros::TimerEvent& event) {
  if(analogInPub.getNumSubscribers() > 0){
    publishAnalogInputStates(ifkit);
  }

  if(digitalInPub.getNumSubscribers() > 0){
    publishDigitalInputStates(ifkit);
  }

  if(digitalOutPub.getNumSubscribers() > 0){
    publishDigitalOutputStates(ifkit);
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "phidgets_interface_kit");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  analogInPub = n.advertise<phidgets_interface_kit::AnalogArray>("/cl4_gpio/analog_in", 1, true);
  digitalInPub = n.advertise<phidgets_interface_kit::DigitalArray>("/cl4_gpio/digital_in", 1, true);
  digitalOutPub = n.advertise<phidgets_interface_kit::DigitalArray>("/cl4_gpio/digital_out", 1, true);
  digitalOutSub = nh.subscribe("cmd_digital_out", 1, onCmdDigitalOut);

  // Load parameters
  int serial_number = -1;
  nh.getParam("serial", serial_number);

  if (attach(ifkit, serial_number)) {

    display_properties(ifkit);
    initialised = true;

    ros::Timer timer = nh.createTimer(ros::Duration(0.033), timerCallback);
    ros::spin();
    disconnect(ifkit);
  }

  return 0;
}
