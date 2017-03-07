#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//services
#include "moveit_utils/MicoController.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_utils/MicoMoveitCartesianPose.h"

#include "segbot_arm_perception/TabletopPerception.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>

//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pcl_ros/impl/transforms.hpp>


//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"



#include "agile_grasp/Grasps.h"

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

// PCL specific includes
//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <geometry_msgs/TwistStamped.h>


#include <segbot_arm_manipulation/arm_utils.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>

using namespace pcl;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

std::vector<PointCloudT::Ptr> detected_objects;
std::vector<shape_msgs::Mesh> result; // mesh objects
std::vector<moveit_msgs::CollisionObject> collision_objects; 
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped start_pose;
geometry_msgs::PoseStamped end_pose;
bool heardPose = false;
bool heardJoinstState = false;

ros::Publisher pub_velocity;
ros::Publisher cloud_pub;
ros::Publisher cloud_grasp_pub;
ros::Publisher pose_array_pub;
ros::Publisher pose_pub;
ros::Publisher pose_pub_target;
ros::Publisher pose_fk_pub;
ros::ServiceClient controller_client;
ros::Publisher pub_rviz;
ros::Publisher pub_min_pt;
ros::Publisher pub_max_pt;
ros::Publisher display_publisher;
moveit_msgs::DisplayTrajectory display_trajectory;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;


/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
  //  ROS_INFO_STREAM(current_pose);
}

void listenForArmData(float rate){
	heardPose = false;
	//heardJoinstState = false;
	ros::Rate r(rate);
	
	while (ros::ok()){
		ros::spinOnce();
		
		if (heardPose)
			return;
		
		r.sleep();
	}
}

template<typename T>
void toPoint(const T &in, geometry_msgs::Point &out)
{
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}




bool service_cb(geometry_msgs::PoseStamped p_target){
    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] Request received!");
    
    moveit_utils::MicoController srv_controller;
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("COLLISION SIZE\n");
    ROS_INFO_STREAM(collision_objects.size());
    
    planning_scene_interface.addCollisionObjects(collision_objects);
    //sleep()2.0);
    group.setPlanningTime(10.0); //5 second maximum for collision computation
    moveit::planning_interface::MoveGroup::Plan my_plan;
    
    geometry_msgs::PoseStamped goal;
    goal.pose.orientation = p_target.pose.orientation;
    goal.pose.position = p_target.pose.position;
    //publish target pose
    //pose_pub_target.publish(p_target);
    //ROS_INFO_STREAM(end_pose);
    
    group.setStartState(*group.getCurrentState());
    group.setPoseTarget(p_target);

    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] starting to plan...");
    bool success = group.plan(my_plan);
    if(success)
		ROS_INFO("planning successful\n");
	else 
		ROS_INFO("not successful :( \n");
		
	group.attachObject(collision_objects[0].id);
	
	if (1)
	{
		ROS_INFO("Visualizing plan 1 (again)");
		display_trajectory.trajectory_start = my_plan.start_state_;
		display_trajectory.trajectory.push_back(my_plan.trajectory_);
		display_publisher.publish(display_trajectory);
		/* Sleep to give Rviz time to visualize the plan. */
		sleep(5.0);
	}

    //call service
    // ROS_INFO("Printing Trajectory \n");
    // ROS_INFO_STREAM(my_plan.trajectory_);
    moveit_utils::MicoController srv;
    srv_controller.request.trajectory = my_plan.trajectory_;

	
	ROS_INFO("CALLING CONTROLLER CLIENT.");
	
    if(controller_client.call(srv_controller)){
       ROS_INFO("Service call sent. Prepare for movement.");
       //res.completed = srv_controller.response.done;
    }
    else {
      ROS_INFO("Service call failed. Is the service running?");
      //res.completed = false;
    }
  
    ros::spinOnce();
    return true;
}

void pressEnter(){
  std::cout << "Press the ENTER key to continue";
  while (std::cin.get() != '\n')
    std::cout << "Please press ENTER\n";
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "collison_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();



  controller_client = nh.serviceClient<moveit_utils::MicoController>("mico_controller");
  //ros::ServiceServer srv = nh.advertiseService("mico_cartesianpose_service", service_cb);

  pose_pub_target = nh.advertise<geometry_msgs::PoseStamped>("/mico_cartesianpose_service/target_pose", 10);

  // //create subscriber to joint angles
  //ros::Subscriber sub_angles = nh.subscribe ("/joint_states", 1, joint_state_cb);

  // //create subscriber to joint torques
  // ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);

  // //create subscriber to tool position topic
  ros::Subscriber sub_tool = nh.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

  // //subscriber for fingers
  //ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
    
  // //subscriber for grasps
  //ros::Subscriber sub_grasps = n.subscribe("/find_grasps/grasps_handles",1, grasps_cb);  
    
  //publish velocities
  //pub_velocity = nh.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
  
  //publish pose array
  //pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/agile_grasp_demo/pose_array", 10);
  
  //publish pose 
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_out", 10);
  pose_fk_pub = nh.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_fk_out", 10);
  pub_rviz = nh.advertise<geometry_msgs::PoseStamped>("/point_rviz", 10);
  pub_min_pt = nh.advertise<geometry_msgs::PoseStamped>("/point_rviz_min", 10);
  pub_max_pt = nh.advertise<geometry_msgs::Point>("/point_rviz_max", 10);
  ros::Publisher pub_box = nh.advertise<visualization_msgs::Marker>("/obstacle_marker", 10 );

  display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  

  
  //debugging publisher
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud_debug", 10);
  cloud_grasp_pub = nh.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);
  
  
  //register ctrl-c
  signal(SIGINT, sig_handler);
  
	//create listener for transforms
	tf::TransformListener tf_listener;

 
  
	segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(nh);
  
	if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table. The end...");
			//exit(1);
	}
	else{
		
		ROS_INFO("Objects Found on the table.");
	}
		
	 ROS_INFO("Demo starting...move the arm to end pose.");
	 pressEnter();
	 listenForArmData(30.0);
	 end_pose = current_pose;
	  
	 ROS_INFO("Demo starting...move the arm to start pose.");
	 pressEnter();
	 listenForArmData(30.0);
	 start_pose = current_pose;
	  
	//select the object with most points as the target object
	int largest_pc_index = -1;
	int largest_num_points = -1;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
			
		int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
		
		if (num_points_i > largest_num_points){
			largest_num_points = num_points_i;
			largest_pc_index = i;
		}
	}
	
	//wait for transform and perform it
	tf_listener.waitForTransform(table_scene.cloud_clusters[largest_pc_index].header.frame_id,"base_link",
    ros::Time(0), ros::Duration(3.0)); 
	
	// Transformed Object - in reference from the base_link
	sensor_msgs::PointCloud2 object_cloud;	
	
	//transform it to base link frame of reference
	pcl_ros::transformPointCloud ("base_link", table_scene.cloud_clusters[largest_pc_index], object_cloud, tf_listener);
		
	segbot_arm_manipulation::closeHand();

  // convert to PCL 
  PointCloudT::Ptr object_i (new PointCloudT);
  pcl::PCLPointCloud2 pc_i;
  pcl_conversions::toPCL(object_cloud,pc_i);
  pcl::fromPCLPointCloud2(pc_i,*object_i);
  

  // get the min and max
  PointT min_pt;
  PointT max_pt;
  ////////////////////////////// 3D MIN MAX ////////////////////////////////////////
  pcl::getMinMax3D(*object_i, min_pt, max_pt); 

  // create a bounding box
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";

  /* The id of the object is used to identify it. */
  collision_object.id = "box";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);

  // Work on these
  primitive.dimensions[0] = max_pt.x - min_pt.x;
  primitive.dimensions[1] = max_pt.y - min_pt.y;
  primitive.dimensions[2] = max_pt.z - min_pt.z;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  min_pt.x;
  box_pose.position.y = min_pt.y;
  box_pose.position.z =  min_pt.z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  

  //fill the detected objects
  detected_objects.clear();
  ROS_INFO("%i", (int)table_scene.cloud_clusters.size());
  for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
    PointCloudT::Ptr object_i (new PointCloudT);
    pcl::PCLPointCloud2 pc_i;
    pcl_conversions::toPCL(table_scene.cloud_clusters.at(i),pc_i);
    pcl::fromPCLPointCloud2(pc_i,*object_i);
    detected_objects.push_back(object_i);
  }
  
  if (detected_objects.size() == 0){
    ROS_INFO("[agile_grasp_demo.cpp] No objects detected");
    //return 1;
  }else{
	  ROS_INFO("Objects Detected!");
  }
  
	// fill up the collison objects vector
	//onTriangulatePcl();
	// publish the pose with the collison objects
  
//visualization_msgs::Marker marker = mark_cluster(object_i, "namespace", 1, 0.0, 1.0, 0.0);

	// pub torviz;
  ROS_INFO("Publishing End Pose");
  pub_rviz.publish(end_pose);
 
 
  ROS_INFO_STREAM(end_pose);
  ROS_INFO("Publishing min and max point to rviz\n");
  //pub_min_pt.publish(min_point);
  //pub_max_pt.publish(max_point); 
  
  
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_i, centroid);
  
  uint32_t shape = visualization_msgs::Marker::CUBE; 
  visualization_msgs::Marker marker; 
  marker.header.frame_id = "base_link"; 
  marker.header.stamp = ros::Time::now(); 
  
  marker.ns = "collision_object"; 
  marker.id = 1; 
  marker.type = shape; 
  marker.action = visualization_msgs::Marker::ADD; 
  
  marker.pose.position.x = centroid[0]; 
  marker.pose.position.y = centroid[1]; 
  marker.pose.position.z = centroid[2]; 
  marker.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
 
 /*
  marker.pose.orientation.x = 0.0; 
  marker.pose.orientation.y = 0.0; 
  marker.pose.orientation.z = 0.0; 
  marker.pose.orientation.w = 1.0;
  */ 
  
  marker.scale.x = (max_pt.x-min_pt.x); 
  marker.scale.y = (max_pt.y-min_pt.y); 
  marker.scale.z = (max_pt.z-min_pt.z); 
  
  if (marker.scale.x ==0) 
      marker.scale.x=0.1; 

  if (marker.scale.y ==0) 
    marker.scale.y=0.1; 

  if (marker.scale.z ==0) 
    marker.scale.z=0.1; 
    
  marker.color.r = 1.0; 
  marker.color.g = 0.5; 
  marker.color.b = 1.5; 
  marker.color.a = 0.5; 

  marker.lifetime = ros::Duration(); 
  ROS_INFO("Marker Printing");
  ROS_INFO_STREAM(marker);
  pub_box.publish(marker);

  //segbot_arm_manipulation::moveToPoseMoveIt(nh,end_pose);
   
  service_cb(end_pose);
  ros::spin();
  //ros::shutdown();
  return 0;
}
