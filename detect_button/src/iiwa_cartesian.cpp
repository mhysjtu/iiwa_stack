#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/MoveToCartesianPoseAction.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetEndpointFrame.h>

static bool setPTPJointSpeedLimits(ros::NodeHandle& nh) {
	ROS_INFO("Setting PTP joint speed limits...");
	ros::ServiceClient setPTPJointSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>("/iiwa/configuration/setPTPJointLimits");
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
	ros::ServiceClient setPTPCartesianSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>("/iiwa/configuration/setPTPCartesianLimits");
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

int main (int argc, char **argv)
{
	ros::init(argc, argv, "cartesian_motion_demo");
	ros::NodeHandle nh;

	// Set speed limit for motions in joint coordinates
	if (!setPTPJointSpeedLimits(nh)) {
		return 1;
	}

	// Set speed limits for motions in cartesian coordinates
	if (!setPTPCartesianSpeedLimits(nh)) {
		return 1;
	}

	// Create the action clients
	// Passing "true" causes the clients to spin their own threads
    actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> cartesianPoseLinClient("/iiwa/action/move_to_cartesian_pose", true); //"/iiwa/action/move_to_cartesian_pose_lin"

	ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
    cartesianPoseLinClient.waitForServer();


	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
    iiwa_msgs::MoveToCartesianPoseGoal cartesianPoseGoal;
	cartesianPoseGoal.cartesian_pose.poseStamped.header.frame_id = "iiwa_link_0";
	cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.x = -0.618675630606;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.y = 0.0902561322359;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.z = -0.574701790009-0.1;//0.3698

    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.x = -0.388212945011;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.y = 0.921380996704;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.z = 0.0184875117455;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.w = -0.00245448822021;

    cartesianPoseGoal.cartesian_pose.redundancy.status = 3;
    cartesianPoseGoal.cartesian_pose.redundancy.turn = 30;
    cartesianPoseGoal.cartesian_pose.redundancy.e1 = -106.92;//if not defined, will move to 0 degree!!!!!

	// Send goal to action server
	cartesianPoseLinClient.sendGoal(cartesianPoseGoal);

	// Wait for the action to finish
	bool finished_before_timeout = cartesianPoseLinClient.waitForResult(ros::Duration(20.0));

	if (!finished_before_timeout) {
		ROS_WARN("iiwa motion timed out - exiting...");
		return 0;
	}
	else if (!cartesianPoseLinClient.getResult()->success) {
		ROS_ERROR("Action execution failed - exiting...");
		return 0;
	}

	ROS_INFO("Done.");

	//exit
	return 0;
}