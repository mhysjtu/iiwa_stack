/**
 * Copyright (C) 2018 Arne Peters - arne.peters@tum.de
 * Technische Universität München
 * Chair for Robotics, Artificial Intelligence and Embedded Systems
 * Fakultät für Informatik / I6, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://www6.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetEndpointFrame.h>

static bool setPTPJointSpeedLimits(ros::NodeHandle& nh) {
	ROS_INFO("Setting PTP joint speed limits...");
	ros::ServiceClient setPTPJointSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>("/iiwa1/configuration/setPTPJointLimits");
	iiwa_msgs::SetPTPJointSpeedLimits jointSpeedLimits;
	jointSpeedLimits.request.joint_relative_velocity = 0.2;
	jointSpeedLimits.request.joint_relative_acceleration = 0.5;
	if (!setPTPJointSpeedLimitsClient.call(jointSpeedLimits)) {
		ROS_ERROR("Service call failed.");
		return false;
	}
	else if (!jointSpeedLimits.response.success) {
		ROS_ERROR_STREAM("Service call returned error: "+jointSpeedLimits.response.error);
		return false;
	}

	ROS_INFO("Done.");
	return true;
}

static bool setPTPCartesianSpeedLimits(ros::NodeHandle& nh) {
	ROS_INFO("Setting PTP Cartesian speed limits...");
	ros::ServiceClient setPTPCartesianSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>("/iiwa1/configuration/setPTPCartesianLimits");
	iiwa_msgs::SetPTPCartesianSpeedLimits cartesianSpeedLimits;
	cartesianSpeedLimits.request.maxCartesianVelocity = 0.5;
	cartesianSpeedLimits.request.maxCartesianAcceleration = 0.5;
	cartesianSpeedLimits.request.maxCartesianJerk = -1.0; // ignore
	cartesianSpeedLimits.request.maxOrientationVelocity = 0.5;
	cartesianSpeedLimits.request.maxOrientationAcceleration = 0.5;
	cartesianSpeedLimits.request.maxOrientationJerk = -1.0; // ignore
	if (!setPTPCartesianSpeedLimitsClient.call(cartesianSpeedLimits)) {
		ROS_ERROR("Failed.");
		return false;
	}
	else if (!cartesianSpeedLimits.response.success) {
		ROS_ERROR_STREAM("Service call returned error: "+cartesianSpeedLimits.response.error);
		return false;
	}

	ROS_INFO("Done.");
	return true;
}

static bool setEndpointFrame(ros::NodeHandle& nh, std::string frameId = "iiwa_link_ee") {
	ROS_INFO_STREAM("Setting endpoint frame to \""<<frameId<<"\"...");
	ros::ServiceClient setEndpointFrameClient = nh.serviceClient<iiwa_msgs::SetEndpointFrame>("/iiwa1/configuration/setEndpointFrame");
	iiwa_msgs::SetEndpointFrame endpointFrame;
	endpointFrame.request.frame_id = frameId;
	if (!setEndpointFrameClient.call(endpointFrame)) {
		ROS_ERROR("Failed.");
		return false;
	}
	else if (!endpointFrame.response.success) {
		ROS_ERROR_STREAM("Service call returned error: "+endpointFrame.response.error);
		return false;
	}

	ROS_INFO("Done.");
	return true;
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "spline_motion_demo");
	ros::NodeHandle nh;

	// Set speed limit for motions in joint coordinates
	if (!setPTPJointSpeedLimits(nh)) {
		return 1;
	}

	// Set speed limits for motions in cartesian coordinates
	if (!setPTPCartesianSpeedLimits(nh)) {
		return 1;
	}

	// Set endpoint frame to flange, so that our Cartesian target coordinates are tool independent
	if (!setEndpointFrame(nh)) {
		return 1;
	}

	// Create the action clients
	// Passing "true" causes the clients to spin their own threads
	actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> jointPositionClient("/iiwa1/action/move_to_joint_position", true);

	ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
	jointPositionClient.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
	iiwa_msgs::MoveToJointPositionGoal jointPositionGoal;
	jointPositionGoal.joint_position.position.a1 =  -1.5558;
	jointPositionGoal.joint_position.position.a2 =  0.1720;
	jointPositionGoal.joint_position.position.a3 =  -0.0938;
	jointPositionGoal.joint_position.position.a4 =  -1.5957;
	jointPositionGoal.joint_position.position.a5 =  -0.0546;
	jointPositionGoal.joint_position.position.a6 =  0.00;//1.2846
	jointPositionGoal.joint_position.position.a7 =  -0.7794;
	// Send goal to action server
	jointPositionClient.sendGoal(jointPositionGoal);

	// Wait for the action to finish
	bool finished_before_timeout = jointPositionClient.waitForResult(ros::Duration(60.0));

	if (!finished_before_timeout) {
		ROS_WARN("iiwa motion timed out - exiting...");
		return 0;
	}
	else if (!jointPositionClient.getResult()->success) {
		ROS_ERROR("Action execution failed - exiting...");
		return 0;
	}

	ROS_INFO("Done.");

	//exit
	return 0;
}
