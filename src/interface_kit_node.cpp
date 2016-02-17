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
#include "interface_kit.h"


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

int InputChangeHandler(CPhidgetInterfaceKitHandle phid, void *usrptr, int Index, int State){
  ROS_INFO("Digital Input: %d > State: %d\n", Index, State);
	return 0;
}

int OutputChangeHandler(CPhidgetInterfaceKitHandle phid, void *usrptr, int Index, int State){
  ROS_INFO("Digital Output: %d > State: %d\n", Index, State);
	return 0;
}

int SensorChangeHandler(CPhidgetInterfaceKitHandle phid, void *usrptr, int Index, int Value){
	ROS_INFO("Sensor: %d > Value: %d\n", Index, Value);
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


	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)phid, ErrorHandler, NULL);

	//Registers a callback that will run if an input changes.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnInputChange_Handler (phid, InputChangeHandler, NULL);

	//Registers a callback that will run if the sensor value changes by more than the OnSensorChange trig-ger.
	//Requires the handle for the IntefaceKit, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnSensorChange_Handler (phid, SensorChangeHandler, NULL);

	//Registers a callback that will run if an output changes.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnOutputChange_Handler (phid, OutputChangeHandler, NULL);

	//open the interfacekit for device connections
	CPhidget_open((CPhidgetHandle)phid, serial_number);

  //Wait for interface kit to be attached
  ROS_INFO("Waiting for interface kit Phidget to be attached....");
  if (serial_number == -1) {
  }
  else {
    ROS_INFO("Waiting for Motor Control HC Phidget %d to be attached....", serial_number);
  }

  int result;
  if((result = CPhidget_waitForAttachment((CPhidgetHandle)phid, 10000)))
	{
    const char *err;
		CPhidget_getErrorDescription(result, &err);
		ROS_ERROR("Problem waiting for interface kit phidget attachment: %s\n", err);
		return false;
	}

  return true;
}


void disconnect(CPhidgetInterfaceKitHandle &phid){
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_interface_kit");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Load parameters
    int serial_number = -1;
    nh.getParam("serial", serial_number);


    if (attach(ifkit, serial_number)) {

      display_properties(ifkit);

      initialised = true;
      ros::Rate loop_rate(1);

      while (ros::ok()) {
          ros::spinOnce();
          loop_rate.sleep();
      }

      disconnect(ifkit);
  }

    return 0;
}
