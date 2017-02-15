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
std::vector<moveit_msgs::CollisionObject> collison_objects; 
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

void addObjectsToCollision(){


   for(int i = 0; i < result.size(); i++){

      moveit_msgs::CollisionObject attached_object;
      //attached_object.link_name = "r_wrist_roll_link";
      /* The header must contain a valid TF frame*/
      attached_object.header.frame_id = "r_wrist_roll_link";
      /* The id of the object */
      attached_object.id = "box";

      /* A default pose */
      geometry_msgs::Pose pose;
      pose.orientation.w = 1.0;

      shape_msgs::Mesh mesh = result.at(i);

      /* Define a box to be attached */
   
      attached_object.meshes.push_back(mesh);
      attached_object.mesh_poses.push_back(pose);
      attached_object.operation = attached_object.ADD;
      collison_objects.push_back(attached_object);
   }
   
}

void polygonMeshToShapeMsg(const PointCloud<PointXYZ> &points,
  const std::vector<Vertices> &triangles,
  shape_msgs::Mesh &mesh)
{
  mesh.vertices.resize(points.points.size());
  for(size_t i=0; i<points.points.size(); i++)
    toPoint(points.points[i], mesh.vertices[i]);

  ROS_INFO("Found %ld polygons", triangles.size());
  BOOST_FOREACH(const Vertices polygon, triangles)
  {
    if(polygon.vertices.size() < 3)
    {
      ROS_WARN("Not enough points in polygon. Ignoring it.");
      continue;
    }

    shape_msgs::MeshTriangle triangle = shape_msgs::MeshTriangle();
    boost::array<uint32_t, 3> xyz = {{polygon.vertices[0], polygon.vertices[1], polygon.vertices[2]}};
    triangle.vertex_indices = xyz;

    mesh.triangles.push_back(shape_msgs::MeshTriangle());
  }
}

void reconstructMesh(const PointCloud<PointXYZ>::ConstPtr &cloud,
  pcl::PointCloud<pcl::PointXYZ> &output_cloud, std::vector<pcl::Vertices> &triangles)
{
  boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
  indices->resize(cloud->points.size ());
  for (size_t i = 0; i < indices->size (); ++i) { (*indices)[i] = i; }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::MovingLeastSquares<pcl::PointXYZ, PointNormal> mls;

  mls.setInputCloud(cloud);
  mls.setIndices(indices);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);
  
  mls.process(*mls_normals);
  
  ConvexHull<PointXYZ> ch;
  
  ch.setInputCloud(mls_points);
  ch.reconstruct(output_cloud, triangles);
}

bool onTriangulatePcl()
{
  ROS_INFO("Service request received");
  
   // make it global
  //std::vector<sensor_msgs::PointCloud2> cloud_raw_vec; // make it global

	std::vector<shape_msgs::Mesh> result;


  // TODO: revomove the selected object
   for ( int i = 0; i < detected_objects.size(); i++)
   {
         sensor_msgs::PointCloud2 cloud_raw;
         pcl::PCLPointCloud2 pc_target;
         pcl::toPCLPointCloud2(*detected_objects.at(i),pc_target);
         pcl_conversions::fromPCL(pc_target, cloud_raw);

         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
         pcl::fromROSMsg(cloud_raw, *cloud);

         pcl::PointCloud<pcl::PointXYZ> out_cloud;
         std::vector<pcl::Vertices> triangles;

         ROS_INFO("Triangulating");
         reconstructMesh(cloud, out_cloud, triangles);
         ROS_INFO("Triangulation done");

         ROS_INFO("Converting to shape message");
         shape_msgs::Mesh mes;
         polygonMeshToShapeMsg(out_cloud, triangles, mes);
      
   }

   // After this collision_object should be filled up with collision objects
   addObjectsToCollision();

  return true;
}

bool service_cb(geometry_msgs::PoseStamped p_target){
    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] Request received!");
    
    moveit_utils::MicoController srv_controller;
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //planning_scene_interface.addCollisionObjects(collison_objects);
    group.setPlanningTime(5.0); //5 second maximum for collision computation
    moveit::planning_interface::MoveGroup::Plan my_plan;
    geometry_msgs::PoseStamped goal;
    goal.pose.orientation = p_target.pose.orientation;
    goal.pose.position = p_target.pose.position;
   
    //publish target pose
    //pose_pub_target.publish(p_target);

   
    group.setPoseTarget(p_target);
    group.setStartState(*group.getCurrentState());

    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] starting to plan...");
    bool success = group.plan(my_plan);
    if(success)
		ROS_INFO("planning successful\n");
	else 
		ROS_INFO("not successful :( \n");
	
	//group.move();
    //call service
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
  
  //debugging publisher
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud_debug", 10);
  cloud_grasp_pub = nh.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);
  
  
  //register ctrl-c
  signal(SIGINT, sig_handler);
  
	//create listener for transforms
	tf::TransformListener tf_listener;

  ROS_INFO("Demo starting...move the arm to end pose.");
  pressEnter();
  listenForArmData(30.0);
  end_pose = current_pose;
  
  ROS_INFO("Demo starting...move the arm to start pose.");
  pressEnter();
  listenForArmData(30.0);
  start_pose = current_pose;
  
	segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(nh);
  
	if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table. The end...");
			//exit(1);
	}
	else{
		
		ROS_INFO("Objects Found on the table.");
	}
		
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
	tf_listener.waitForTransform(table_scene.cloud_clusters[largest_pc_index].header.frame_id,"base_link",ros::Time(0), ros::Duration(3.0)); 
	
	// Transformed Object - in reference from the base_link
	sensor_msgs::PointCloud2 object_cloud;	
	
	//transform it to base link frame of reference
	pcl_ros::transformPointCloud ("base_link", table_scene.cloud_clusters[largest_pc_index], object_cloud, tf_listener);
		

	
	segbot_arm_manipulation::closeHand();
  
  

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

 
	
 
	//tranform pose into arm frame of reference
	//tf::TransformListener listener;
	//listener.waitForTransform(p_target.header.frame_id, "mico_api_origin", ros::Time(0), ros::Duration(3.0));
	//listener.transformPose("mico_api_origin", p_target, p_target);
			
	// fill up the collison objects vector
	onTriangulatePcl();
	// publish the pose with the collison objects
  
	
	// pub torviz;
	ROS_INFO("Moving to start pose now\n");
	pub_rviz.publish(end_pose);
	//segbot_arm_manipulation::moveToPoseMoveIt(nh,end_pose);
   
	service_cb(end_pose);
  ros::spin();
  //ros::shutdown();
  return 0;
}
