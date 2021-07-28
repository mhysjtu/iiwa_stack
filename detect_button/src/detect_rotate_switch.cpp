#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/MoveToCartesianPoseAction.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetEndpointFrame.h>
#include <iiwa_msgs/ConfigureControlMode.h>
#include <iiwa_msgs/ControlMode.h>
#include <iiwa_msgs/CartesianQuantity.h>
#include <gripper_msgs/gripper_control.h>

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
	cartesianSpeedLimits.request.maxCartesianVelocity = 0.05;
	cartesianSpeedLimits.request.maxCartesianAcceleration = 0.1;
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

int main (int argc, char** argv) {
    ros::init(argc, argv, "detect_rotate_switch");
    ros::NodeHandle nh;

    //gripper setup
    ros::ServiceClient gripper_client = nh.serviceClient<gripper_msgs::gripper_control>("gripper1/control_service");
    gripper_msgs::gripper_control srv;

    //manipulator setup
    actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> cartesianPoseLinClient("/iiwa/action/move_to_cartesian_pose_lin", true);

    ros::ServiceClient iiwa_config_client = nh.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");
    iiwa_msgs::ConfigureControlMode pos_srv;
    pos_srv.request.control_mode = iiwa_msgs::ControlMode::POSITION_CONTROL;

    iiwa_msgs::ConfigureControlMode cartImp_srv;
    cartImp_srv.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
    iiwa_msgs::CartesianQuantity cart_quan;
    cart_quan.a = 300;
    cart_quan.b = 300;
    cart_quan.c = 300;
    cart_quan.x = 2000;
    cart_quan.y = 100;
    cart_quan.z = 100;
    cartImp_srv.request.cartesian_impedance.cartesian_stiffness=cart_quan;

    // start to move
	if (!setPTPJointSpeedLimits(nh)) {
		return 1;
	}
	if (!setPTPCartesianSpeedLimits(nh)) {
		return 1;
	}

    //move to pose 1
    ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
    cartesianPoseLinClient.waitForServer();

	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
    // !!!!Must be read when set param to GripperY!!!!
	iiwa_msgs::MoveToCartesianPoseGoal cartesianPoseGoal;

    //panel 11 circular button
    double B_x = -0.709378420243;
    double B_y = 0.187311499775;
    double B_z = -0.810505626049-0.15;
    double B_qx = 0.665554702282;
    double B_qy = 0.313011213968;
    double B_qz = -0.625066810563;
    double B_qw = 0.261442982746;
    double B_e1 = -1.88815693812;

    // panel 12 rotate
    double R_x = -0.676551433401;
    double R_y = 0.143461990321;
    double R_z = -0.791340224647-0.15;
    double R_qx = 0.665503561497;
    double R_qy = 0.312941744364;
    double R_qz = -0.625022201714;
    double R_qw = 0.261762685414;
    double R_status = 1;
    double R_turn = 22;
    double R_e1 = -1.96905614879;

	cartesianPoseGoal.cartesian_pose.poseStamped.header.frame_id = "iiwa_link_0";
	cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.x = B_x;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.y = B_y;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.z = B_z;//0.3698

    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.x = B_qx;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.y = B_qy;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.z = B_qz;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.w = B_qw;

    cartesianPoseGoal.cartesian_pose.redundancy.status = 1;
    cartesianPoseGoal.cartesian_pose.redundancy.turn = 22;
    cartesianPoseGoal.cartesian_pose.redundancy.e1 = B_e1;//if not defined, will move to 0 degree!!!!!


    // cartesianPoseGoal.cartesian_pose.poseStamped.header.frame_id = "iiwa_link_0";
	// cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.x = R_x;
    // cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.y = R_y;
    // cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.z = R_z;

    // cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.x = R_qx;
    // cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.y = R_qy;
    // cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.z = R_qz;
    // cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.w = R_qw;

    // cartesianPoseGoal.cartesian_pose.redundancy.status = 1;
    // cartesianPoseGoal.cartesian_pose.redundancy.turn = 22;
    // cartesianPoseGoal.cartesian_pose.redundancy.e1 = R_e1;



    cartesianPoseGoal.force_threshold = 4;//4 good; 15 for button press

	// Send goal to action server
	cartesianPoseLinClient.sendGoal(cartesianPoseGoal);

    // Wait for the action to finish
	bool finished_before_timeout = cartesianPoseLinClient.waitForResult(ros::Duration(10.0));

	if (!finished_before_timeout) {
		ROS_WARN("iiwa motion timed out - exiting...");
		return 0;
	}
	else if (!cartesianPoseLinClient.getResult()->success) {
		ROS_ERROR("Action execution failed - exiting...");
		return 0;
	}



    return 0;

    // //gripper rotate
    // srv.request.type  = 4;
    // srv.request.value = 90;
    // if(gripper_client.call(srv)) {
    //     ROS_INFO("call gripper1 success");
    // }else {
    //     ROS_ERROR("Failed to call service gripper1/control_service");
    // }

    //change to iiwa impedence mode
    if(iiwa_config_client.call(cartImp_srv)) {
        ROS_INFO("call impedence mode success");
    }else {
        ROS_ERROR("Failed to call impedence mode service");
    }
    ROS_INFO("waiting to close gripper");
    int tmp=0;
    std::cin>>tmp;
    std::cin.get();
    //gripper close
    srv.request.type  = 1;
    srv.request.value = 50;
    if(gripper_client.call(srv)) {
        ROS_INFO("call gripper1 success");
    }else {
        ROS_ERROR("Failed to call service gripper1/control_service");
    }

    //gripper rotate
    int rotate_angle=90;
    ROS_INFO("input rotate angle 60");
    std::cin>>rotate_angle;
    srv.request.type  = 4;
    srv.request.value = rotate_angle;
    if(gripper_client.call(srv)) {
        ROS_INFO("call gripper1 success");
    }else {
        ROS_ERROR("Failed to call service gripper1/control_service");
    }
    
    ROS_INFO("input rotate angle 30");
    std::cin>>rotate_angle;
    srv.request.type  = 4;
    srv.request.value = rotate_angle;
    if(gripper_client.call(srv)) {
        ROS_INFO("call gripper1 success");
    }else {
        ROS_ERROR("Failed to call service gripper1/control_service");
    }

    ROS_INFO("input rotate angle 90");
    std::cin>>rotate_angle;
    srv.request.type  = 4;
    srv.request.value = rotate_angle;
    if(gripper_client.call(srv)) {
        ROS_INFO("call gripper1 success");
    }else {
        ROS_ERROR("Failed to call service gripper1/control_service");
    }
    

    //gripper open
    ROS_INFO("open gripper");
    std::cin>>tmp;
    srv.request.type  = 1;
    srv.request.value = 1000;
    if(gripper_client.call(srv)) {
        ROS_INFO("call gripper1 success");
    }else {
        ROS_ERROR("Failed to call service gripper1/control_service");
    }

    ROS_INFO("change mode to position");
    std::cin>>tmp;
    //change to iiwa position mode
    if(iiwa_config_client.call(pos_srv)) {
        ROS_INFO("call position mode success");
    }else {
        ROS_ERROR("Failed to call position mode service");
    }

    return 0;
}