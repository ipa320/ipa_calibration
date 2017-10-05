These instructions for running the robot calibration software can be viewed in better quality at http://wiki.ros.org/robotino_calibration . The page also contains instructions for alternative localization and calibration methods.

# Camera base calibration using a box for localization and a checkerboard

1. Conduct a laser scanner to base calibration, see README.md in https://github.com/federico-b/squirrel_nav/tree/laser_odom_calibration/laser_odom_calibration.

2. Measure laser scanner height and put in the number into the properties.urdf.xacro file of your robot which is located, e.g., at squirrel_robotino/robotino_bringup/robots/uibk-robotino/urdf/properties.urdf.xacro.
There search for the block:
  <!-- hokuyo mount positions | relative to base_link -->
  <property name="hokuyo_x" value="0.131740483"/>
  <property name="hokuyo_y" value="0.00937244242"/>
  <property name="hokuyo_z" value="0.102"/> <!-- not used, apprx -->
  <property name="hokuyo_roll" value="0.0"/>
  <property name="hokuyo_pitch" value="0.0"/>
  <property name="hokuyo_yaw" value="-0.0536545473"/>
and enter the measure at hokuyo_z.
Optimally, you measure your laser scanner height using a level that is shifted downwards a ruler until it becomes visible in the laser scan. Repeat the procedure from below: shifting the level upwards until visible in the laser scan. Take the average of both measured heights.

3. Mount a calibration checkerboard (e.g. from robotino_calibration/common/files/checkerboard_a3_6x4_5cm.pdf in A3 size) horizontally on a wall (use a level to be precise), approximately at the height of Robotino's Kinect (at the moment this is left upper chessboard calibration corner approx. at 80cm height above ground). There should not be any object around, try to find a flat wall segment of at least 2.0m length and place robotino and the checkerboard in the center.
Enter the properties of the used calibration pattern (grid size, square side length) into file 'squirrel_robotino/robotino_calibration/ros/launch/camera_base_calibration_params.yaml' at the lines:
 ```
 ### checkerboard parameters
 # side length of the chessboard squares
 chessboard_cell_size: 0.05
 # number of checkerboard calibration points (in x- and y-direction), i.e. those points where 4 squares meet
 chessboard_pattern_size: [6,4]
 ```

4. Place a box directly below the leftmost line of checkerboard calibration points (i.e. where 4 squares meet). Align the left side of the box with the leftmost calibration points. The box must stand apart from the wall by more than 11 cm. The box is used to localize the calibration pattern with the laser scanner.
Measure the vertical distance between the laser scanner plane and the upper left checkerboard point and insert the number into file 'squirrel_robotino/robotino_calibration/ros/launch/checkerboard_localisation.launch' at the line:
```
<node pkg="tf" type="static_transform_publisher" name="static_transform_publisher_landmark_marker" output="screen" args="0.0 -0.7 0 0 0 0  landmark_reference marker 100"/>
```
Replace the -0.7 at the y-offset with your measured number (all other numbers should be 0). Use a negative measure.

5. Check whether all coordinate frame names are correctly chosen is file 'squirrel_robotino/robotino_calibration/ros/launch/camera_base_calibration_params.yaml' at the lines:
 ```
 # link names for the robot coordinate systems
 # string
 torso_lower_frame: "base_pan_link"		# fixed frame at lower torso end (i.e. this coordinate system should not turn if different pan angles are commanded)
 torso_upper_frame: "tilt_link"			# last link of torso chain [the transformations between torso_lower_frame and torso_upper_frame should be available from tf]
 camera_frame: "kinect_link"				# camera frame with fixed transform to torso_upper_frame [transform from torso_upper_frame to camera will be calibrated by this program]
 camera_optical_frame: "kinect_rgb_optical_frame"	# this is the camera coordinate system which refers to the color image sensor [the transformations between camera_frame and camera_optical_frame should be available from tf]
 base_frame: "base_link"					# the robot base frame [the transformation between laser scanner and base should be accomplished before, the transform from base_frame to torso_lower_framewill be calibrated by this program]
 checkerboard_frame: "checkerboard"		# do not modify, this is the coordinate system fixed in the upper left checkerboard calibration corner (where 4 squares meet), it will be published to tf via the checkerboard_localisation program
 ```
 
6. (Optional) Start 'roslaunch robotino_calibration checkerboard_localisation.launch' and open RViz. The add the Marker topic /wall_marker which displays the detected wall (green line) and the box in front of the wall (blue points). Make sure that both are detected correctly.

7. Fill in good initial values for the transformations to estimate in file 'squirrel_robotino/robotino_calibration/ros/launch/camera_base_calibration_params.yaml':
e.g.
 ```
 ### initial values for transformation estimates
 # insert the values as x, y, z, yaw (rot around z), pitch (rot around y), roll (rot around x)
 # the transform from base_frame to torso_lower_frame
 T_base_to_torso_lower_initial: [0.25, 0, 0.5, 0.0, 0.0, 0.0]

 # the transform from torso_upper_frame to camera_frame
 T_torso_upper_to_camera_initial: [0.0, 0.065, 0.0, 0.0, 0.0, -1.57]
 ```

8. Fill in a good amount (i.e. >30) of robot configurations for observing the checkerboard measured relative to the landmark_reference_nav coordinate system that is similarly aligned as the robot's base_link facing the checkerboard. This goes into file 'squirrel_robotino/robotino_calibration/ros/launch/camera_base_calibration_params.yaml':
e.g.
 ```
 ### checkerboard observation positions for capturing calibration images
 # list of robot configurations for observing the checkerboard measured relative to the landmark_reference_nav coordinate system that is similarly aligned as the robot's base_link when facing the marker (e.g. checkerboard)
 # includes 5 parameters per entry: robot pose: x, y, phi and torso: pan, tilt
 robot_configurations: [-1.5, -0.17, 0, 0.15, 0.25,
                        -1.5, -0.17, 0, 0.0, 0.3,
                        -1.5, -0.17, 0, -0.15, 0.3,
                        -1.5, -0.17, 0, -0.3, 0.3,
                        -1.5, -0.17, 0, -0.5, 0.3,
                        -1.5, -0.17, 0, 0.15, 0.05,
                        -1.5, -0.17, 0, 0.0, 0.05,
                        -1.5, -0.17, 0, -0.15, 0.05,
                        -1.5, -0.17, 0, -0.3, 0.05,
                        -1.5, -0.17, 0, -0.5, 0.05,
                        -1.5, -0.17, 0, 0.15, -0.2,
                        -1.5, -0.17, 0, 0.0, -0.2,
                        -1.5, -0.17, 0, -0.15, -0.2,
                        -1.5, -0.17, 0, -0.35, -0.2,
                        -1.5, -0.17, 0, -0.5, -0.2,
                        -1.0, -0.17, 0, 0.0, 0.2,
                        -1.0, -0.17, 0, -0.2, 0.2,
                        -1.0, -0.17, 0, -0.45, 0.2,
                        -1.0, -0.17, 0, 0.0, 0.05,
                        -1.0, -0.17, 0, -0.2, 0.05,
                        -1.0, -0.17, 0, -0.45, 0.05,
                        -1.0, -0.17, 0, 0.0, -0.15,
                        -1.0, -0.17, 0, -0.2, -0.15,
                        -1.0, -0.17, 0, -0.45, -0.15,
                        -0.85, -0.17, 0, 0.0, 0.15,
                        -0.85, -0.17, 0, -0.15, 0.15,
                        -0.85, -0.17, 0, -0.35, 0.2,
                        -0.85, -0.17, 0, 0.0, 0.05,
                        -0.85, -0.17, 0, -0.15, 0.05,
                        -0.85, -0.17, 0, -0.35, 0.05,
                        -0.85, -0.17, 0, 0.0, -0.1,
                        -0.85, -0.17, 0, -0.15, -0.1,
                        -0.85, -0.17, 0, -0.35, -0.1]
 ```
 
9. Start 'roslaunch robotino_calibration camera_base_calibration.launch'. The robot will drive into different observation positions and move the torso to capture calibration images of the checkerboard at the wall. After image recording, the intrinsic camera calibration and the extrinsic transform estimation will start and ouput their results in the end. During image acquisition, all image and transformation data is also stored to disk so that calibration can run again offline with the load_images parameter set in file 'squirrel_robotino/robotino_calibration/ros/launch/camera_base_calibration_params.yaml'.

10. Replace the extrinsic calibration parameters output by the program in your 'squirrel_robotino/robotino_bringup/robots/xyz_robotino/urdf/properties.urdf.xacro':
e.g.
 ```
  <!-- base_neck_link mount positions | camera base calibration | relative to base_link -->
  <property name="shell_x" value="0.308614"/>
  <property name="shell_y" value="-0.00660354"/>
  <property name="shell_z" value="0.669155"/>
  <property name="shell_roll" value="0.0205246"/>
  <property name="shell_pitch" value="-0.00419423"/>
  <property name="shell_yaw" value="0.208381"/>

  <!-- kinect mount positions | camera base calibration | relative to neck_tilt_link -->
  <property name="kinect_x" value="0.00633317"/>
  <property name="kinect_y" value="0.0586273"/>
  <property name="kinect_z" value="0.010865"/>
  <property name="kinect_roll" value="-1.50705"/>
  <property name="kinect_pitch" value="0.0150564"/>
  <property name="kinect_yaw" value="0.0080777"/>
 ```
