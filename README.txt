Main README:
1. Set up a computer with Ubuntu 24.04 and ROS2 Jazzy (install the full desktop version)
2. Source /opt/ros/jazzy/setup.sh
3. Install dependencies using rosdep:
    rosdep install --from-paths src --ignore-src -r -y
4. Build with symlink install
    colon build --symlink-install 
5. Source the install:
    source install/setup.sh
5. Run the kuka launch example by doing:
    ros2 launch kuka kuka_launch.py








Personal log:
I'm creating a virtual environment here. We'll need to source this every time for installing packages. It has catkin_pkg in it.

Remember that UTC -> UCT symlink exists and is jugar


I'm creating a seperate package (kuka_moveit). Here I generated a config file with ros2 launch moveit_setup_assistant setup_assistant.launch.py


In order to get gazebo to spawn in the kuka I need to run:

export GZ_SIM_RESOURCE_PATH=$(pwd)/src/:



I've enabled the URDF to have ROS2_control capibilities

For kuka_moveit sometimes it will stop working, you ahve to define the max_joint_accelerations to be 1.0



Currently ros2_control is having issues. I'm going to create a package and build that from source for ros. I have to source that package sometimes, so I'm going to probably add that as a command to be executed first.



I'm using this to load the controllers <plugin>mock_components/GenericSystem</plugin>

Furthermore, I've messed a little with the gz_workspace, asking it to export an interface. So if I have lots of issues i might have to reinstall that.





I've disabled this guy in my-demo.launch.py
# mappings={
#     "ros2_control_hardware_type": LaunchConfiguration(
#         "ros2_control_hardware_type"
#     )
# },

export GZ_SIM_SYSTEM_PLUGIN_PATH=install/gz_ros2_control/lib -> how to make the controllers load



#TODO: Fix robot state publisher + broadcaster issue

# Add SDF file 






I found a good resource here: https://robotics.stackexchange.com/questions/103881/gazebo-plugin-location-and-documentation






