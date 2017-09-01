**Source:** The client code can be found [here](https://github.com/poeffie/roboy_ik/tree/master/src)

##Main
The client has almost no modifications to a standard client, apart from saving the pose coordinates as floats with the function `atof()`. Further explanations can be found on the [official ROS Service tutorial](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)

```
int main(int argc, char **argv)
{
  ros::init(argc, argv, "roboy_ik_client");
  
  if (argc != 4)
  {
    ROS_INFO("usage: send position X Y Z");
    return 1;
  }
  ros::NodeHandle n;
  
  ros::ServiceClient client = n.serviceClient<roboy_ik::InverseKinematics>("roboy_ik");
  roboy_ik::InverseKinematics srv;
  srv.request.a = atof(argv[1]);
  srv.request.b = atof(argv[2]);
  srv.request.c = atof(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("Inverse Kinematics Success: %s", (int)srv.response.sum ? "true" : "false");
  }
  else
  {
    ROS_ERROR("Failed to call inverse kinematics");
    return 1;
  }

  return 0;
}
```
