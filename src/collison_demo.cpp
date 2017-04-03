#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> //???
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

#include <ros/ros.h>
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

// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <segbot_arm_manipulation/arm_utils.h>

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

ros::Publisher pub_velocity;
ros::ServiceClient controller_client;
ros::Publisher pub_rviz;
ros::Publisher display_publisher;
moveit_msgs::DisplayTrajectory display_trajectory;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;


/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
    current_pose = msg;
    heardPose = true;
}

void listenForArmData(float rate) {
    heardPose = false;
    ros::Rate r(rate);
    while (ros::ok()) {
        ros::spinOnce();
	if (heardPose) 
            return;
	r.sleep();
    }
}

template<typename T>
void toPoint(const T &in, geometry_msgs::Point &out) {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
}

bool service_cb(geometry_msgs::PoseStamped p_target) {
    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] Request received!");
    
    moveit_utils::MicoController srv_controller;
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    ROS_INFO("Collision size:");
    ROS_INFO_STREAM(collision_objects.size());

    sleep(2.0);
    planning_scene_interface.addCollisionObjects(collision_objects);
    sleep(2.0);

    group.setPlanningTime(20.0); //5 second maximum for collision computation

    moveit::planning_interface::MoveGroup::Plan my_plan;
    
    geometry_msgs::PoseStamped goal;
    goal.pose.orientation = p_target.pose.orientation;
    goal.pose.position = p_target.pose.position;
    
    group.setStartState(*group.getCurrentState());
    group.setPoseTarget(p_target);

    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] starting to plan...");
    bool success = group.plan(my_plan);
    if (success)
        ROS_INFO("Planning successful\n");
    else {
        ROS_WARN("Planning unsuccessful. Please rerun.\n");
        exit(1);
    }
		
    group.attachObject(collision_objects[0].id);
	
    ROS_INFO("Visualizing plan. Check Rviz to see trajectory.");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
		
    //Sleep to give Rviz time to visualize the plan.
    sleep(5.0);
    
    moveit_utils::MicoController srv;
    srv_controller.request.trajectory = my_plan.trajectory_;

	
    ROS_INFO("Calling controller client");
	
    if (controller_client.call(srv_controller)) {
        ROS_INFO("Service call sent. Prepare for movement.");
    }
    else {
        ROS_WARN("Service call failed. Is the service running? Please rerun.");
        exit(1);
    }
  
    ros::spinOnce();
    return true;
}

void pressEnter() {
    std::cout << "Press the ENTER key to continue";
    while (std::cin.get() != '\n')
        std::cout << "Please press ENTER\n";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "collison_demo");
    ros::NodeHandle nh;

    controller_client = nh.serviceClient<moveit_utils::MicoController>("mico_controller");
    
    //publish pose 
    pub_rviz = nh.advertise<geometry_msgs::PoseStamped>("/point_rviz", 10);
    ros::Publisher pub_box = nh.advertise<visualization_msgs::Marker>("/obstacle_marker", 10 );

    display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
		
	ros::Subscriber sub_tool = nh.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
    //register ctrl-c
    signal(SIGINT, sig_handler);
  
    //create listener for transforms
    tf::TransformListener tf_listener;

    segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(nh);
  
    if ((int)table_scene.cloud_clusters.size() == 0) {
        ROS_WARN("No objects found on table. Exiting. Please add objects and rerun.");
	    exit(1);
    } else {
	    ROS_INFO("Objects found on the table.");
    }
	  
    //select the object with most points as the target object
    int largest_pc_index = -1;
    int largest_num_points = -1;
    for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++) {
        int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
	    if (num_points_i > largest_num_points) {
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
  
    //3D Min/Max
    pcl::getMinMax3D(*object_i, min_pt, max_pt); 
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*object_i, centroid);

    // create a bounding box
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";

    //Id of object used to identify it
    collision_object.id = "box";

    //Define a box to add to the world
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);

    primitive.dimensions[0] = max_pt.x - min_pt.x;
    primitive.dimensions[1] = max_pt.y - min_pt.y;
    primitive.dimensions[2] = max_pt.z - min_pt.z;

    //Pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
    box_pose.position.x = centroid[0];
    box_pose.position.y = centroid[1];
    box_pose.position.z = centroid[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    //fill the detected objects
    detected_objects.clear();
    ROS_INFO_STREAM((int)table_scene.cloud_clusters.size());
  
    for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++) {
        PointCloudT::Ptr object_i (new PointCloudT);
        pcl::PCLPointCloud2 pc_i;
        pcl_conversions::toPCL(table_scene.cloud_clusters.at(i),pc_i);
        pcl::fromPCLPointCloud2(pc_i,*object_i);
        detected_objects.push_back(object_i);
    } 
  
    if (detected_objects.size() == 0) {
        ROS_INFO("[agile_grasp_demo.cpp] No objects detected");
        exit(1);
    } else {
	    ROS_INFO("Objects Detected!");
    }
  

    //Creating marker
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
 
    marker.scale.x = (max_pt.x - min_pt.x); 
    marker.scale.y = (max_pt.y - min_pt.y); 
    marker.scale.z = (max_pt.z - min_pt.z); 
  
    if (marker.scale.x == 0)  marker.scale.x = 0.1; 
    if (marker.scale.y == 0)  marker.scale.y = 0.1; 
    if (marker.scale.z == 0)  marker.scale.z = 0.1; 
    
    marker.color.r = 1.0; 
    marker.color.g = 0.5; 
    marker.color.b = 1.5; 
    marker.color.a = 0.5; 

    marker.lifetime = ros::Duration(); 
    
    ROS_INFO("Marker printed. Check RViz to see which object has been chosen as the obstacle.");
    pub_box.publish(marker);

	
    ROS_INFO("Demo starting...");
    ROS_INFO("Move the arm to end pose.");
    pressEnter();
    ROS_INFO("Mo start pose.");
    listenForArmData(30.0);
    end_pose = current_pose;

    ROS_INFO("Move the arm to start pose.");
    pressEnter();
    listenForArmData(30.0);
    start_pose = current_pose;
  
    ROS_INFO("Publishing end pose. Check Rviz to see end pose.");
    pub_rviz.publish(end_pose);

   
    service_cb(end_pose);
    ROS_INFO("Demo has ended. Ctrl-C to exit.");
    ros::spin();
    return 0;
}
