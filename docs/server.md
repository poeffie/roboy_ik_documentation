# Server Implementation
In the following chapters, the code is described step by step, splitting it into compact blocks to improve readability. The heading will in general be left out, as it only contains *includes*, which does not need further explanations. We start here with the ROS server node, the initializing part for inverse kinematics calulations. The server works together with a so called client, which build together a ROS service.

**Source:** The server code can be found [here](https://github.com/poeffie/roboy_ik/tree/master/src)

## Main
`roboy_ik_server` is the name of the server node and `roboy_ik` the package name. We initialize a common server as it is taught on the [official ROS Service tutorial](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
```
int main(int argc, char **argv)
{
  ros::init(argc, argv, "roboy_ik_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("roboy_ik", call_ik);
  ROS_INFO("Ready to send position request.");
  ros::spin();

  return 0;
}
```

## Callback
The callback will be executed as soon as a request is detected.
`InverseKinematics` is the name of the .srv file.
In the first line we are checking a the sum of `req.a`, `req.b`, and `req.c`. These values are the x, y and z coordinates of the desired position, that is requested by the client. This statement is always true, as the position will for sure not overshoot this value. So basically this is still subject of implementation and should be replaced in the future by a reasonable Roboy state query, that provides Roboy not to execute inverse kinematics, if other subtasks are blocking.
```
bool call_ik(roboy_ik::InverseKinematics::Request  &req,
         roboy_ik::InverseKinematics::Response &res)
{
//TODO: Find a reasonable state query to replace this if / else condition
  if((req.a + req.b + req.c) < 10000.0) {
  	res.sum = true;
  	ofstream pose_file;
    pose_file.open ("/home/offi/catkin_ws/src/roboy_ik/src/pose.txt");
    pose_file << std::fixed << std::setprecision(5) << req.a <<endl;
    pose_file << std::fixed << std::setprecision(5) << req.b <<endl;
    pose_file << std::fixed << std::setprecision(5) << req.c;
    pose_file.close();
  }
  else {
	res.sum = false;
  }
  ROS_INFO("request: x=%lf, y=%lf, z=%lf", (double)req.a, (double)req.b, (double)req.c);
  ROS_INFO("sending back response: [%d]", (bool)res.sum);
  if(res.sum == true) {
    system("/home/offi/catkin_ws/src/roboy_ik/src/run.sh");
  }
  
  return true;
}
```
If Roboy is allowed to execute the IK, what should basically be done is that the publisher node will receive this values and start calculating the inverse kinematics and finally, if successful, publish the joint trajectory. The upcomming problem here is that the Publisher Node can't be started from another node. It is not possible to define two nodes within one .cpp file, especially not a server and a publisher node.

To call the publisher from a running node and also send a response to the client, which includes information wheather the IK calculations were successful, I came up with the following solution, that may be not optimal but sufficient for prototyping.

First of all, the pose is stored locally in the file `pose.txt`. This is the connecting link to share information between service and publisher. Afterwards the system function calls `run.sh`, a .bash file that executes the publisher via .launch file. The code is the following:
```
#!/bin/bash  
roslaunch roboy_ik motion_planning_api_tutorial.launch
echo "Successfully executed motion planning"
```
As the publisher node has to terminate completely after finishing the calculations, one needs to edit the `<node>` tags in the `motion_planning_api.launch`, found in `/catkin_ws/src/roboy_ik/launch`, to contain required="true" and make respawn="false".

