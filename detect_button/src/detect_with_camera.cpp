#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

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
#include <gripper_msgs/yolo_icp_pose.h>


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

void rosPoseToEigenMatrix4d(geometry_msgs::Pose &input, Eigen::Matrix4d &output) {
    Eigen::Quaterniond q(input.orientation.w, input.orientation.x,input.orientation.y, input.orientation.z); //w,x,y,z
    Eigen::Matrix3d R = q.toRotationMatrix();
    double p[3] = {input.position.x, input.position.y, input.position.z}; //x,y,z
    for(unsigned int i=0;i<3;i++){
        for(unsigned int j=0;j<3;j++){
            output(i,j) = R(i,j);
        }
    }
    for(unsigned int i=0;i<3;i++){
        output(i,3) = p[i];
    }
}

void setCartesianGoal(geometry_msgs::PoseStamped &input, iiwa_msgs::MoveToCartesianPoseGoal &output) {
    output.cartesian_pose.poseStamped.header.frame_id = "iiwa_link_0";
	output.cartesian_pose.poseStamped.pose.position.x = input.pose.position.x;
    output.cartesian_pose.poseStamped.pose.position.y = input.pose.position.y;
    output.cartesian_pose.poseStamped.pose.position.z = input.pose.position.z;

    output.cartesian_pose.poseStamped.pose.orientation.x = input.pose.orientation.x;
    output.cartesian_pose.poseStamped.pose.orientation.y = input.pose.orientation.y;
    output.cartesian_pose.poseStamped.pose.orientation.z = input.pose.orientation.z;
    output.cartesian_pose.poseStamped.pose.orientation.w = input.pose.orientation.w;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "detect with camera");
    ros::NodeHandle nh;


    //----------setup----------
    // //gripper setup
    // ros::ServiceClient gripper_client = nh.serviceClient<gripper_msgs::gripper_control>("gripper1/control_service");
    // gripper_msgs::gripper_control srv;

    //manipulator motion setup
    actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> cartesianPoseLinClient("/iiwa/action/move_to_cartesian_pose_lin", true);
    iiwa_msgs::MoveToCartesianPoseGoal cartesianPoseGoal;

    // //iiwa control mode setup
    // ros::ServiceClient iiwa_config_client = nh.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");
    // iiwa_msgs::ConfigureControlMode pos_srv;
    // pos_srv.request.control_mode = iiwa_msgs::ControlMode::POSITION_CONTROL;
    // iiwa_msgs::ConfigureControlMode cartImp_srv;
    // cartImp_srv.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
    // iiwa_msgs::CartesianQuantity cart_quan;
    // cart_quan.a = 300;
    // cart_quan.b = 300;
    // cart_quan.c = 300;
    // cart_quan.x = 2000;
    // cart_quan.y = 100;
    // cart_quan.z = 100;
    // cartImp_srv.request.cartesian_impedance.cartesian_stiffness=cart_quan;

    // yolo icp service
    ros::ServiceClient yolo_icp_client = nh.serviceClient<gripper_msgs::yolo_icp_pose>("yolo_icp_pose");
    gripper_msgs::yolo_icp_pose icp_srv;

    // start to move
	if (!setPTPJointSpeedLimits(nh)) {
		return 1;
	}
	if (!setPTPCartesianSpeedLimits(nh)) {
		return 1;
	}
    //------------------------------------

    //------move to observe position------
    ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
    cartesianPoseLinClient.waitForServer();

	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
	cartesianPoseGoal.cartesian_pose.poseStamped.header.frame_id = "iiwa_link_0";
	cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.x = ;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.y = ;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.z = ;

    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.x = ;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.y = ;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.z = ;
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.w = ;

    cartesianPoseGoal.cartesian_pose.redundancy.status = 3;
    cartesianPoseGoal.cartesian_pose.redundancy.turn = 30;
    cartesianPoseGoal.cartesian_pose.redundancy.e1 = -106.92;
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
    ROS_INFO("start to detect panel");
    //------------------------------------

    //---------- detect panel ------------
    icp_srv.request.name = 0;
    geometry_msgs::Pose pose_panel_to_cam;
    if(yolo_icp_client.call(icp_srv)) {
        ROS_INFO("call impedence mode success");
        pose_panel_to_cam = icp_srv.response.pose;
    }else {
        ROS_ERROR("Failed to call impedence mode service");
    }

    // change pose to Eigen
    Eigen::Matrix4d pose_panel_in_cam;
    rosPoseToEigenMatrix4d(pose_panel_to_cam, pose_panel_in_cam);
    //------------------------------------

    //---------- convert pose ------------
    // TODO: ee in world, get by reading topic
    iiwa_msgs::MoveToCartesianPoseGoal pose_ee_to_world;
    pose_ee_to_world.cartesian_pose.poseStamped.header.frame_id = "iiwa_link_0";
	pose_ee_to_world.cartesian_pose.poseStamped.pose.position.x = -0.629775819826;
    pose_ee_to_world.cartesian_pose.poseStamped.pose.position.y = 0.191053385425;
    pose_ee_to_world.cartesian_pose.poseStamped.pose.position.z = -0.793568845206-0.08;

    pose_ee_to_world.cartesian_pose.poseStamped.pose.orientation.x = 0.664715707302;
    pose_ee_to_world.cartesian_pose.poseStamped.pose.orientation.y = 0.313576490453;
    pose_ee_to_world.cartesian_pose.poseStamped.pose.orientation.z = -0.625816750218;
    pose_ee_to_world.cartesian_pose.poseStamped.pose.orientation.w = 0.261105704606;

    pose_ee_to_world.cartesian_pose.redundancy.status = 3;
    pose_ee_to_world.cartesian_pose.redundancy.turn = 30;
    pose_ee_to_world.cartesian_pose.redundancy.e1 = -106.92;

    //pose to Eigen
    Eigen::Matrix4d pose_ee_in_world;
    rosPoseToEigenMatrix4d(pose_ee_to_world.cartesian_pose.poseStamped.pose, pose_ee_in_world);
    
    // TODO: hand eye calibration result
    Eigen::Quaterniond cam_in_ee_q(1.000, -0.002, -0.003, 0.004); //w,x,y,z
    Eigen::Matrix3d cam_in_ee_r = cam_in_ee_q.toRotationMatrix();
    double cam_in_ee_p[3] = {-0.018, -0.034, 0.018}; //x,y,z
    Eigen::Matrix4d pose_cam_in_ee;
    for(unsigned int i=0;i<3;i++){
        for(unsigned int j=0;j<3;j++){
            pose_cam_in_ee(i,j) = cam_in_ee_r(i,j);
        }
    }
    for(unsigned int i=0;i<3;i++){
        pose_cam_in_ee(i,3) = cam_in_ee_p[i];
    }   


    Eigen::Matrix4d pose_panel_in_world;
    pose_panel_in_world = pose_ee_in_world * pose_cam_in_ee * pose_panel_in_cam;


    //target ee in world
    double target_ee_in_world_p[3]; //x,y,Z
    target_ee_in_world_p[0] = pose_panel_in_world(0,3);
    target_ee_in_world_p[1] = pose_panel_in_world(1,3);
    target_ee_in_world_p[2] = pose_panel_in_world(2,3)+0.5; // TODO

    Eigen::Matrix3d target_ee_in_world_r;
    for(unsigned int i=0;i<3;i++){
        for(unsigned int j=0;j<3;j++){
            target_ee_in_world_r(i,j) = pose_panel_in_world(i,j);
        }
    }    
    Eigen::Quaterniond target_ee_in_world_q(target_ee_in_world_r); //w,x,y,z
    //------------------------------------

    //----------move iiwa----------
    //move to target above
    ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
    cartesianPoseLinClient.waitForServer();

	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
	cartesianPoseGoal.cartesian_pose.poseStamped.header.frame_id = "iiwa_link_0";
	cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.x = target_ee_in_world_p[0];
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.y = target_ee_in_world_p[1];
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.z = target_ee_in_world_p[2];

    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.x = target_ee_in_world_q.x();
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.y = target_ee_in_world_q.y();
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.z = target_ee_in_world_q.z();
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.w = target_ee_in_world_q.w();

    cartesianPoseGoal.cartesian_pose.redundancy.status = pose_ee_to_world.cartesian_pose.redundancy.status;
    cartesianPoseGoal.cartesian_pose.redundancy.turn = pose_ee_to_world.cartesian_pose.redundancy.turn;
    cartesianPoseGoal.cartesian_pose.redundancy.e1 = pose_ee_to_world.cartesian_pose.redundancy.e1;
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
    //------------------------------------

    return 0;
/*

    //----------move iiwa----------
    //move to contact panel
    ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
    cartesianPoseLinClient.waitForServer();

	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
	cartesianPoseGoal.cartesian_pose.poseStamped.header.frame_id = "iiwa_link_0";
	cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.x = target_ee_in_world_p[0];
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.y = target_ee_in_world_p[1];
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.position.z = target_ee_in_world_p[2]-0.15;

    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.x = target_ee_in_world_q.x();
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.y = target_ee_in_world_q.y();
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.z = target_ee_in_world_q.z();
    cartesianPoseGoal.cartesian_pose.poseStamped.pose.orientation.w = target_ee_in_world_q.w();

    cartesianPoseGoal.cartesian_pose.redundancy.status = pose_ee_to_world.cartesian_pose.redundancy.status;
    cartesianPoseGoal.cartesian_pose.redundancy.turn = pose_ee_to_world.cartesian_pose.redundancy.turn;
    cartesianPoseGoal.cartesian_pose.redundancy.e1 = pose_ee_to_world.cartesian_pose.redundancy.e1;
    cartesianPoseGoal.force_threshold = 4;

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



    //----------detect with gripper----------

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
    */
}