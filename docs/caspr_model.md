# Model Creation - Tutorial
In the following, I am going to describe how to create a CASPR model manually, without using the GUI. This is helpful to understand the principles of CASPR models and for CASPROS model creation, as CASPROS uses the same files like CASPR.

1.) In your CASPR-master folder go to /data/model_config

2.) Create a new Folder,  e.g. "myModel"

3.) Create three .xml files, where "myModel" can be replaced by the model name, that you wish. 

* myModel_bodies.xml
* myModel_cables.xml
* myModel_trajectories.xml

## Body
4.) Edit myModel_bodies.xml first. The most Basic Model looks like this:
```
<?xml version="1.0" encoding="utf-8"?>
<links display_range="-1.0 1.0 -1.0 1.0 -1.0 1.0" view_angle="-37 32">
  <link_rigid num="1" name="Upper Arm">
    <joint type="S_EULER_XYZ" q_initial="0 0 0"/>
    <physical>
      <mass>1.0</mass>
      <com_location>0.0 0.25 0.0</com_location>
      <end_location>0.0 0.5 0.0</end_location>
      <inertia ref="com">
         <Ixx>0.1</Ixx>
         <Iyy>0.1</Iyy>
         <Izz>0.1</Izz>
         <Ixy>0.1</Ixy>
         <Ixz>0.1</Ixz>
         <Iyz>0.1</Iyz>
      </inertia>
    </physical>
    <parent>
      <num>0</num>
      <location>0.0 0.0 0.0</location>
    </parent>
  </link_rigid>
</links>
```
The name of the body can be set by the attribute *name="NAME"* in the `<link_rigid></link_rigid>` element.
In general, this code will create a link with its starting postion in `<location></loaction>` element, here also the coordinate origin.
`<com_location></com_loaction>` describes the center of mass location in x-y-z coordinates.
`<end_location></end_location>` describes the end location of the link.
`<joint_type></joint_type>` describes the rotation axis of the body.

**NOTE:** In this basic example, as well as the following ones, the mass, inertia mass as well as locations are set to arbitrary values. for example here it is mass 1.0 and 0.1 for each inertia mass element. 
Furthermore, the link always rotates around the point of its starting position. These are the possible rotational settings:

```
R_X % Revolute in X
R_Y % Revolute in Y
R_Z % Revolute in Z
U_XY % Universal with xy-euler
U_YZ % Universal with yz-euler %%
P_XY % Translational joint in XY plane
PLANAR_XY % Planar in XY plane
PLANAR_YZ % Planar in YZ plane
PLANAR_XZ % Planar in XZ plane %%
S_EULER_XYZ % Spherical xyz-euler
S_FIXED_XYZ % Spherical xyz-fixed
S_QUATERNION % Spherical joint using quaternion
T_XYZ % Translational joint XYZ
SPATIAL_QUATERNION % T_XYZ + SPHERICAL
SPATIAL_EULER_XYZ % T_XYZ + S_EULER_XYZ
```

Inerta matrix and mass will be set depending on the measurements of the robot link, that should be simulated.
last but not least is the link number, which is important for attaching cables.
It is written in the `<num></num>` element.

## Trajectory
6.) Edit the myModel_trajectorys.xml. The basic trajectory with two joint waypoints could look like this:

```
<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE trajectories [
<!ATTLIST linear_spline_trajectory id ID #REQUIRED>
<!ATTLIST cubic_spline_trajectory id ID #REQUIRED>
<!ATTLIST quintic_spline_trajectory id ID #REQUIRED>
]>
<trajectories>
  <quintic_spline_trajectory id="traj_test" time_definition="absolute" time_step="0.00667">
    <points>
      <point>
        <q>0.0</q>
        <q_dot>0.0</q_dot>
        <q_ddot>0.0</q_ddot>
      </point>
      <point time="1">
        <q>0.3</q>
        <q_dot>0.0</q_dot>
        <q_ddot>0.0</q_ddot>
      </point>
    </points>
  </quintic_spline_trajectory>
</trajectories>
```

In this case the robot has one planar joint, that moves from 0 degree to 0.3 rad after one second. If the robot has more degrees of freedom, due to more or more complex joints, like spherical ones, one will have to add more q values.
The q values are always just seperated by a blank space.
The name of the trajectory can be set by the element attribute *id="NAME"*.
Example:
```
<q>0.0 0.0 0.0</q>
<q_dot>0.0 0.0 0.0</q_dot>
<q_ddot>0.0 0.0 0.0</q_ddot>
```
## Cables
7.) Edit the myModel_cables.xml. For one cable, this file could look like the following:
```
<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE cables [
<!ATTLIST cable_set id ID #REQUIRED>
]>
<cables default_cable_set="CS1">
  <cable_set id="CS1">
    <cable_ideal name="cable 1" attachment_reference="com">
      <properties>
        <force_min>5</force_min>
        <force_max>35</force_max>
      </properties>
      <attachments>
        <attachment>
          <link>0</link>
          <location>-0.106066 0.020 -0.106066</location>
        </attachment>
        <attachment>
          <link>1</link>
          <location>-0.025 0.1180 0.0</location>
        </attachment>
      </attachments>
    </cable_ideal>
  </cable_set>
</cables>
```

In general, the interesting part of a cable are the attachement points. One attaches a cable between two links. The links are described by their `<num></num>` value in the .xml file for the body.
The name of the cable set in this case is "CS1".

# Integrating the model
After creating the .xml files, one has to link the new model to CASPR.
This is done by editing "models_list.csv" in directory `/data/model_config`.
One simple adds another line. In our case, the line could be like this:
`My Model,/MyModel/,myModel_bodies.xml,myModel_cables.xml,myModel_trajectories.xml,`

NOTE: The line is internally seperated by commas. The first element of the line is the name of our model, this is also the name, that will be displayed in CASPR. The second element is the path to the model files, followed by three elements, the .xml files for body, cables and trajectory.
After saving all files, you are done and successfully created and integrated a new CASPR model!


# PaBiLegs Example
As an example for a robot model I want to reference here my [PaBiLegs model](https://github.com/poeffie/roboy_ik_additional_files). The model can be used by creating a new folder in the `CASPR-master/data/model_config/models/` location, called `PaBiLegs`. After copying the three necessary .xml files, edit `models_list.csv` by adding this line:
`PaBiLegs,/PaBiLegs/,PaBiLegs_bodies.xml,PaBiLegs_cables.xml,PaBiLegs_trajectories.xml,`

