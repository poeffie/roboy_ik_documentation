**Source:** The publisher code can be found [here](https://github.com/poeffie/roboy_ik/tree/master/src)

# Publisher Implementation
The publisher does the main calculations. As it only has a main function, the code is split up in blocks for better readability. This implementation is based on the official [MoveIt! Motion Planners Tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/motion_planning_api_tutorial.html)

## Robot Model Init
We start by initializing the publisher node, called `motion`. The second step is instantiating a RobotModelLoader object, that looks up the robot description on the ROS parameter server and constructs a RobotModel. What robot model is used as *"robot_description"* can be seen and edited in the `motion_planning_api.launch` file. This process will be described in a later chapter.
```	
// ROS Initilisation
ros::init(argc, argv, "motion");
ros::AsyncSpinner spinner(1);
spinner.start();
ros::NodeHandle node_handle("~");
// MoveIt! Robot Model Initialisation
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
```

## Planning Init
The PlanningScene maintains the state of the whole world and is created by the `robot_model`. Afterwards the planner is loaded by ROS plugin library. We wil discuss the planner in the IKFast chapter.
``` 
planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
planning_interface::PlannerManagerPtr planner_instance;
std::string planner_plugin_name;
```

## Loading Planning Plugin
Planners are plugins in MoveIt! and you can use the ROS pluginlib interface to load any planner that you want to use.
```
// We get the name of planning plugin we want to load
// from the ROS param server, and then load the planner
// making sure to catch all exceptions.
if (!node_handle.getParam("planning_plugin", planner_plugin_name))
  ROS_FATAL_STREAM("Could not find planner plugin name");
try
{
  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
}
catch (pluginlib::PluginlibException& ex)
{
  ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
}
try
{
  planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
    ROS_FATAL_STREAM("Could not initialize planner instance");
  ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
}
catch (pluginlib::PluginlibException& ex)
{
  const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
  std::stringstream ss;
  for (std::size_t i = 0; i < classes.size(); ++i)
    ss << classes[i] << " ";
  ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
}
```

## Wait to startup Rviz
Sleeping a little to allow time to startup rviz, etc.
```
ros::WallDuration sleep_time(15.0);
sleep_time.sleep();
```

## Definitions
Defining variables, that will be used later.
```
planning_interface::MotionPlanRequest req;
planning_interface::MotionPlanResponse res;
planning_interface::PlanningContextPtr context;
moveit_msgs::Constraints pose_goal;
moveit_msgs::MotionPlanResponse response;
moveit_msgs::DisplayTrajectory display_trajectory;
```

## Get Pose
We receive the pose by opening `pose_file.txt` and reading out all lines. We do not consider the orientation in this implementation. But this can easyily be modified by making use of the fourth array storage element of `poses`.
``` 
std::string poses[4];
ifstream pose_file("/home/offi/catkin_ws/src/roboy_ik/src/pose.txt");
int i = 0;
if(!pose_file) 
{
  cout<<"Error opening output file"<<endl;
  system("pause");
  return -1;
}
ROS_INFO("Goal Position (x, y, z):");
while(!pose_file.eof())
{
  getline(pose_file, poses[i], '\n');
  ROS_INFO("%s", poses[i].c_str());
}
```

## Set Pose
Now we write the pose coordinates to a gemoetry message, that will be used for the motion plan request. The `pose.header.frame_id` is the coordinate frame of the robot model to which the pose will reference to. Chosing `odom_combined` ensures that the end effector moves to the desired pose coordnates, considering point `odom_combined` (0, 0, 0) as reference frame.
```
// Sample Pose: 0.03394;  -0.17347;   -0.37346;
geometry_msgs::PoseStamped pose;
pose.header.frame_id = "odom_combined";
pose.pose.position.x = atof(poses[0].c_str());
pose.pose.position.y = atof(poses[1].c_str());
pose.pose.position.z = atof(poses[2].c_str());
pose.pose.orientation.w = 1.0;;


std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.1);
 
pose_goal = kinematic_constraints::constructGoalConstraints("pabi_legs__link_0_0", pose, tolerance_pose, tolerance_angle);
req.group_name = "leg";
req.goal_constraints.push_back(pose_goal);
```
  
## Calculating Inverse Kinematics
Now we construct and call a planning context that encapsulate the scene, the request and the response.
```
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
if (res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  return 0;
}
```

## Visualizing
Finally we visualize the trajectory, which also means publishing the joint trajectory via the topic `/move_group/display_planned_path`.
```
/* Visualize the trajectory */
ROS_INFO("Visualizing the IK trajectory");
  
ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
res.getMessage(response);

display_trajectory.trajectory_start = response.trajectory_start;
display_trajectory.trajectory.push_back(response.trajectory);
display_publisher.publish(display_trajectory);
```

