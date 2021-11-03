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
	ros::ServiceClient setPTPJointSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>("/iiwa2/configuration/setPTPJointLimits");
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
	ros::ServiceClient setPTPCartesianSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>("/iiwa2/configuration/setPTPCartesianLimits");
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
	ros::init(argc, argv, "cartesian_rel_motion_test");
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
    actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> cartesianPoseRelClient("/iiwa2/action/move_to_cartesian_pose", true); //"/iiwa/action/move_to_cartesian_pose_lin"

	ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
    cartesianPoseRelClient.waitForServer();


	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
    iiwa_msgs::MoveToCartesianPoseGoal cartesianPoseRelGoal;
	cartesianPoseRelGoal.cartesian_pose.poseStamped.header.frame_id = "iiwa2_link_0";
	cartesianPoseRelGoal.cartesian_pose.poseStamped.pose.position.x = 0.021; //坐标系是相对TCP而言的
    cartesianPoseRelGoal.cartesian_pose.poseStamped.pose.position.y = 0;
    cartesianPoseRelGoal.cartesian_pose.poseStamped.pose.position.z = 0;

	cartesianPoseRelGoal.cartesian_pose.poseStamped.pose.orientation.x = 0;
    cartesianPoseRelGoal.cartesian_pose.poseStamped.pose.orientation.y = 0;
    cartesianPoseRelGoal.cartesian_pose.poseStamped.pose.orientation.z = 0;
    cartesianPoseRelGoal.cartesian_pose.poseStamped.pose.orientation.w = 1;

    cartesianPoseRelGoal.force_threshold = 5.0; 

	// Send goal to action server
	cartesianPoseRelClient.sendGoal(cartesianPoseRelGoal);

	// Wait for the action to finish
	bool finished_before_timeout = cartesianPoseRelClient.waitForResult(ros::Duration(20.0));

	if (!finished_before_timeout) {
		ROS_WARN("iiwa motion timed out - exiting...");
		return 0;
	}
	else if (!cartesianPoseRelClient.getResult()->success) {
		ROS_ERROR("Action execution failed - exiting...");
		return 0;
	}

	ROS_INFO("Done.");

	//exit
	return 0;
}