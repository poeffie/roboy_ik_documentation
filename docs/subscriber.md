**Source:** The subscriber code can be found [here](https://github.com/poeffie/roboy_ik/tree/master/src)

**Note:** There have to be done some fixes marked with TODO, as the code, like it is presented here, was optimized for the PaBiLegs model on CASPR. The model there uses 6 joints for visualizing the geometry of the legs better. But actually it only needs two links and two joints, as our planning group for PaBiLegs Ik and therefore the trajectory output only outputs two joint values per waypoint. So I made some modifications. These can be undone easily or one can use the universally applicable subscriber code, added in the roboy_ik package and named traj_sub_uni.cpp.

# Main
We start with the main function. The main function is completely basic and subscribes to the topic `/move_group/display_planned_path`, that is published by the publisher node `motion`.
```
int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_trajectory");
  ros::NodeHandle n;
  ros::Subscriber traj_sub = n.subscribe("/move_group/display_planned_path", 1000, traj_sub_callback);
  ros::spin();
  return 0;
}
```
# Callback
As soon as message from the topic is sent out and recognized, the code jumps into the subscriber callback. Here we basically read out the trajectory values and store them into an 2D array. The array has a predefined size. `NUM_WAYPOINTS` is the number of waypoints, the trajectory consits of. This value varies, based on how much the new pose differs from the original one. This means that one has to read out the length of the trajectory to define the array size. Unfortunately this problem could not be solved yet, so that it is still subjet of implementation.
```
void traj_sub_callback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
    const trajectory_msgs::JointTrajectory traj = msg->trajectory[0].joint_trajectory;
    // TODO: Get length of trajectoy / number of waypoints and replace NUM_WAYPOINTS 
    // with the resulting variable
    double joint_traj_array[NUM_WAYPOINTS][NUM_JOINTS];
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        for(int j = 0; j < NUM_JOINTS; j++) {
            // TODO: remove this, if all joints are used. This condition is for experimental
            // purposes. Keep it, if just e.g. the first two will be used, as it is the case
            // for the PaBiLegs demonstration
            if (j >= 2) {
				joint_traj_array[i][j] = 0.0;
			}
			else {
				joint_traj_array[i][j] = traj.points[i].positions[j];
			}
            ROS_INFO("Array value: [%f]", joint_traj_array[i][j]);
        }
        ROS_INFO(" ");
    }
    /**
     * Call a function to write the trajectory values into a .xml file.
     */
    write_joint_trajectory_xml(joint_traj_array);

}

```
# Creating XML
Writing the values from the array into an .xml file is done in this function. In general we use the opensource C++ plugin *tinyXML* here to generate our file. Instead of splitting up this comparably long function into multiple blocks, as it was done for the publisher node, we keep here the formatation and explain the functionality mainly inside of the code by some comments.

```
void write_joint_trajectory_xml (double q_array[NUM_WAYPOINTS][NUM_JOINTS]) {
        int i = 0;

		ROS_INFO("IN CALLBACk222!!!");
        TiXmlDocument doc;

        TiXmlElement * root = new TiXmlElement( "trajectories" );
        doc.LinkEndChild( root );

        TiXmlElement * cxn = new TiXmlElement( "quintic_spline_trajectory" );
        root->LinkEndChild( cxn );
        cxn->SetAttribute("id", "traj_test2");
        cxn->SetAttribute("time_definition", "absolute");
        cxn->SetAttribute("time_step", "0.00667");

        TiXmlElement * pts = new TiXmlElement( "points" );
        cxn->LinkEndChild( pts );

        TiXmlElement * point = new TiXmlElement( "point" );
        pts->LinkEndChild( point );
        /**
         * In this state of development I have not yet implemented the conversion from array
         * data to joint strings. This will be the next step. So far, I just set all the joint
         * values to 0.
         */
        TiXmlElement * q = new TiXmlElement( "q" );
        point->LinkEndChild( q );
        TiXmlText * q_values = new TiXmlText( get_joint_string(q_array, i) );
        q->LinkEndChild( q_values );

        TiXmlElement * q_dot = new TiXmlElement( "q_dot" );
        point->LinkEndChild( q_dot );
        TiXmlText * q_dot_values = new TiXmlText( "0.0 0.0 0.0 0.0 0.0 0.0"  );
        q_dot->LinkEndChild( q_dot_values );

        TiXmlElement * q_ddot = new TiXmlElement( "q_ddot" );
        point->LinkEndChild( q_ddot );
        TiXmlText * q_ddot_values = new TiXmlText( "0.0 0.0 0.0 0.0 0.0 0.0"  );
        q_ddot->LinkEndChild( q_ddot_values );
        /**
         * If the trajectory consits of more than one point, this loop will be called.
         * The first point of a trajectory and the following ones differ by the point-attribute
         * time, that is set here.
         */
        if (IS_TRAJECTORY == 1)
        {
                for (double i = 1; i < NUM_WAYPOINTS; i++) {
                        TiXmlElement * point = new TiXmlElement( "point" );
                        pts->LinkEndChild( point );

                        TiXmlElement * q = new TiXmlElement( "q" );
                        point->LinkEndChild( q );
                        point->SetAttribute("time", to_string(i/10));
                        TiXmlText * q_values = new TiXmlText( get_joint_string(q_array, i)  );
                        q->LinkEndChild( q_values );

                        TiXmlElement * q_dot = new TiXmlElement( "q_dot" );
                        point->LinkEndChild( q_dot );
                        // TODO: replace "0.0 0.0 0.0 0.0 0.0 0.0" by get_joint_string(q_array, i)
                        // if you want to have full functionalty. This is just for experimental
                        // purposes, as the PaBiLegs CASPR model has 6 joints, instead of 2.
                        TiXmlText * q_dot_values = new TiXmlText( "0.0 0.0 0.0 0.0 0.0 0.0"  );
                        q_dot->LinkEndChild( q_dot_values );

                        TiXmlElement * q_ddot = new TiXmlElement( "q_ddot" );
                        point->LinkEndChild( q_ddot );
                        // TODO: replace "0.0 0.0 0.0 0.0 0.0 0.0" by get_joint_string(q_array, i)
                        // if you want to have full functionalty. This is just for experimental
                        // purposes, as the PaBiLegs CASPR model has 6 joints, instead of 2.
                        TiXmlText * q_ddot_values = new TiXmlText( "0.0 0.0 0.0 0.0 0.0 0.0"  );
                        q_ddot->LinkEndChild( q_ddot_values );
                }
        }
        /**
         * tinyXML does not parse DOCTYPE elements. CASPR/OS needs a DOCTYPE declaration.
         * The doctype declaration never changes for any trajectory, so I created an XML file, called
         * head, containing the top of a trajectory XML file, that never changes.
         * trajo.xml is the part of the trajectory that contains the individual joint angles
         * we read from ROS MoveIt. Therefore it has to be generated seperately and combined
         * afterwards with the head.xml
         * trajo_combined.xml is the final usable trajectory file for CASPR / CASPROS
         */

        // TODO: change the file path for trajo.xml, head.xml, trajo_combined.xml as you wish.
        doc.SaveFile( "/home/offi/catkin_ws/src/roboy_ik/src/trajo.xml" );
        ifstream file1( "/home/offi/catkin_ws/src/roboy_ik/src/head.xml" ) ;
        ifstream file2( "/home/offi/catkin_ws/src/roboy_ik/src/trajo.xml" ) ;
        ofstream combined_file( "/home/offi/catkin_ws/src/roboy_ik/src/trajo_combined.xml" );
        combined_file << file1.rdbuf() << file2.rdbuf();
        
        ifstream  src("/home/offi/catkin_ws/src/roboy_ik/src/trajo_combined.xml", std::ios::binary);
        // TODO: change the file path to the CASPR model accordingly
        ofstream  dst("/home/offi/CASPR-master/data/model_config/models/PaBiLegs/PaBiLegs_trajectories.xml",   std::ios::binary);

        dst << src.rdbuf();
        ROS_INFO("Trajectory successfully created!");
}
```

# Get Joint String
This function generates a joint string out of the array values, as it can be seen in the code above, where we have 6 joints and each joint has the value of zero: `"0.0 0.0 0.0 0.0 0.0 0.0"`. The values can, but do not absolutely need to be seperated by commas.
```
string get_joint_string(double q_array[NUM_WAYPOINTS][NUM_JOINTS], int wp){
    stringstream stream;
    for (int i = 0; i < NUM_JOINTS; i++){
		stream << q_array[wp][i];
		if (i != NUM_JOINTS-1) {
			stream << ", ";
		}
    }
    return stream.str();
}
```
