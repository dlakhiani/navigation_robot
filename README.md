# Simulating ROS Navigation in Gazebo

Simulated environment in which programmers are able to interact and experiment with 3D models at their fingertips. Being able to test algorithms, design robots, monitor systems of networks are essential to a succesful project. With Gazebo, a programmer is able to make use of an incredible physics engine, high-quality graphical interfaces and robust programming to design real tools before they are created.

_This blog uses Ubuntu 18.04 with ROS Melodic and Gazebo 9. We assume that you have ROS and Gazebo already installed on your local system._

## Installing Nvidia

This project makes use of Nvidia hardware to lower resource consumption and make the simulation run smoothly!

### Nvidia Driver

```bash
sudo apt-get install nvidia-driver-470
sudo apt-get install nvidia-cuda-toolkit
nvcc --version
```

### Runtime Container

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
nvidia-smi
```

## Launch in Docker

Before we begin this tutorial, you can view the full project in docker!

- Make sure you have [docker](https://docs.docker.com/engine/install/) and [docker-compose](https://docs.docker.com/compose/install/) on your OS.
  - `docker -v`
  - `docker-compose --version`
- Open a new terminal. Clone the repo below:
  ```bash
  cd ~
  git clone https://github.com/dlakhiani/ros-navigation-local.git
  cd ros-navigation-local/src
  xhost local:docker
  docker-compose -f docker-compose.nvidia.yml build
  docker-compose -f docker-compose.nvidia.yml up
  ```
  > _Please do be patient when loading Gazebo, as it will take a bit of time due to it being a graphical client._
- To control the robot, open a new terminal and type:
  ```bash
  docker exec -it src_ros-develop_1 bash
  source devel/setup.bash
  roslaunch turtlebot_teleop keyboard_teleop.launch
  ```

## Workspace and project

- Feel free to skip this if you already have setup a workspace, if not:
  ```bash
  source /opt/ros/$ROS_DISTRO/setup.bash
  mkdir -p ~/sim_ws/src
  cd ~/sim_ws/src
  catkin_init_workspace
  cd ..
  catkin_make
  source ~/sim_ws/devel/setup.bash
  ```
- Lets create the project for this tutorial:
  ```bash
  cd ~/sim_ws/src
  catkin_create_pkg navigation_robot gazebo_ros urdf
  ```

## Gazebo Simulation Box

- Make sure you have [gazebo installed](http://gazebosim.org/tutorials?cat=install) on your OS.
- Launch an empty world of gazebo on your local system.

  - `gazebo`
    ![empty_world](images/gazebo_empty.png)

- You now have an empty world in gazebo.

## World files

A world in gazebo is just another word for a simulated environment for your experiments. Programmers can evaluate and test their robot in difficult or dangerous scenarios without any harm to the robot themselves.

- An equivalent method of creating an empty world in gazebo is using the `.world` file.
  ```bash
  cd ~/sim_ws/src/navigation_robot
  mkdir world
  cd world
  touch empty_world.world
  ```
- To make an empty world in gazebo using `.world`, use the following template:

  ```xml
  <?xml version="1.0" ?>

  <!-- simulation description format - describes models in xml -->
  <sdf version="1.5">
    <world name="default">

      <!-- sky -->
      <include>
        <uri>model://sun</uri>
      </include>

      <!-- ground -->
      <include>
        <uri>model://ground_plane</uri>
      </include>

    </world>
  </sdf>
  ```

- Save this under `empty_world.world`
- You can now load this file using:
  - `gazebo empty_world.world`

## Adding objects to the world

- Using the gazebo model database, adding objects are simple through a world file.
- Lets add a grey wall to our `empty_world.world` file above:

  ```xml
  <include>
    <uri>model://grey_wall</uri>
    <pose>0 0 0 0 0 3.14</pose>
    <name>wall_1</name>
  </include>
  ```

  - the term **uri** is used to specify the model we want to spawn into the gazebo environment, [find more here.](https://github.com/osrf/gazebo_models)
  - the term **pose** (_x y z roll pitch yaw_) determines the model's location and orientation in the gazebo environment.
  - the term **name** allows you to spawn several of the same model into the envionment, by just changing the model's name to a unique one.

- There are several more properties you can add to models in gazebo, [see here for more details.](http://gazebosim.org/tutorials?tut=build_model)

## Spawn a Robot in Gazebo

- Robots are like any other model, having multiple links, joints, plugins and, sensors. In this tutorial, we will spawn a **TurtleBot** !
  ```bash
  cd ~
  git clone https://github.com/dlakhiani/ros-navigation-local.git
  cd ros-navigation-local
  catkin_make
  source devel/setup.bash
  roslaunch turtlebot_navigation_gazebo main.launch
  ```
  ![turtlebot_in_world](images/turtlebot_tutorial.png)

## Control the Robot

- Now that we have spawned a robot into a world in gazebo, lets try moving it! We can do this by using **teleoperation**!
  - `roslaunch turtlebot_teleop keyboard_teleop.launch`
    ![turtlebot_teleop](images/turtlebot_teleop.png)
- Enjoy!

## Monitor Robot

### Rosnode

- ROS makes use of nodes to control the robot's publishers, subscribers and, other connections.
- It is used to computer and display debug information to the user, [read more here.](http://wiki.ros.org/rosnode)
  - `rosnode list` displays currently running rosnodes
    ![rosnode_list_turtlebot](images/rosnode_list_turtlebot.png)
  - `rosnode info /gazebo_gui` provides the connections of the given rosnode
    ![rosnode_info_gazebo_gui](images/rosnode_info_gazebo_gui.png)

### Rostopic

- ROS nodes use topics to publish, subscribe, and send messages to/from the robot from/to the system network.
- With this, we are also able to view the type and content of the messages being sent to the user, [read more here.](http://wiki.ros.org/rostopic)
  - `rostopic list` displays currently running rostopics
    ![rostopic_list_turtlebot](images/rostopic_list_turtlebot.png)
  - `rostopic info /cmd_vel` provides the type of message the topic communicates with, along with any publishers/subscribers that are linked to it
    ![rostopic_info_cmd_vel](images/rostopic_info_cmd_vel.png)
    > _By using this topic (cmd_vel), we were able to control the TurtleBot._
  - `rostopic echo /odom` prints the messages that is sent and received by the topic
    ![rostopic_echo_odom](images/rostopic_echo_odom.png)

### Rviz

- Rviz is a program interface that allows the user to visually monitor simulated objects and their topics.
- It is incredibly helpful when experimenting with custom models and topics, [learn how to use it here!](http://wiki.ros.org/rviz/UserGuide)
  - `roslaunch turtlebot_rviz_launchers view_model.launch`
    ![turtlebot_rviz](images/turtlebot_rviz.png)

### RQT

- `rqt_topic` helps the user view all the currently running rostopic's messages at a glance.
  - `rosrun rqt_topic rqt_topic`
    ![turtlebot_topic](images/turtlebot_topic.png)
- `rqt_graph` provides the user with a flowchart view of the connections between the currently running rostopics and rosnodes.
  - `rosrun rqt_graph rqt_graph`
    ![turtlebot_graph](images/turtlebot_graph.png)

---

<br>

All robot simulations make use of **XML**, especially the `.urdf` extension. The **URDF** (Universal Robot Description Format) model is a collection of **XML** files that describe a robot's physical description. These files are used by **ROS** (Robot Operating System) to tell the computer what the robot actually looks like in real life.

## Launch in Docker

For the customized simulation, you can view it in docker too!

- Make sure you have [docker](https://docs.docker.com/engine/install/) and [docker-compose](https://docs.docker.com/compose/install/) on your OS.
  - `docker -v`
  - `docker-compose --version`
- Open a new terminal. Clone the repo below:
  ```bash
  cd ~
  git clone https://github.com/dlakhiani/iris_model.git
  cd iris_model
  xhost local:docker
  docker-compose build
  docker-compose up ros-develop-gmapping
  ```
  > _Please do be patient when loading Gazebo, as it will take a bit of time due to it being a graphical client._
- To control the robot, open a new terminal and type:
  ```bash
  docker exec -it iris_model_ros-develop_1 bash
  source devel/setup.bash
  rosrun iris_model teleop_twist_key.py
  ```

## Customize World

Earlier we worked with `.world` files to generate an environment for our robot. This time, we are going to customize it:

```bash
cd ~/sim_ws/src/navigation_robot
```

- Lets add some walls to our `empty_world.world`:

  ```xml
  <include>
    <uri>model://grey_wall</uri>
    <name>wall_1</name>
    <pose>5.752843 -5.123935 0 0 0 3.14</pose>
  </include>
  <include>
    <uri>model://grey_wall</uri>
    <name>wall_2</name>
    <pose>5.752843 -3.123935 0 0 0 3.14</pose>
  </include>
  <include>
    <uri>model://grey_wall</uri>
    <name>wall_3</name>
    <pose>5.752843 -1.123935 0 0 0 3.14</pose>
  </include>
  <include>
    <uri>model://grey_wall</uri>
    <name>wall_4</name>
    <pose>5.752843 1.123935 0 0 0 3.14</pose>
  </include>
  <include>
    <uri>model://grey_wall</uri>
    <name>wall_5</name>
    <pose>5.752843 3.123935 0 0 0 3.14</pose>
  </include>
  <include>
    <uri>model://grey_wall</uri>
    <name>wall_6</name>
    <pose>5.752843 5.123935 0 0 0 3.14</pose>
  </include>
  ```

- Save this under `empty_world.world`
- As it is a good practice to use a `.launch` file, we will create one now!
  - **LAUNCH** files are **XML** extensions that provide a convenient way to start up multiple nodes and a master, as well as other initialization factors.
    ```bash
    cd ~/sim_ws/src/navigation_robot
    mkdir launch
    cd launch
    touch world.launch
    ```
- Now add the below to the `world.launch` file:

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>

  <launch>
    <arg name="gui" default="true" />
    <arg name="world" default="$(find navigation_robot)/world/empty_world.world" />

    <!-- include gazebo_ros launcher -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(arg world)" />
        <arg name="gui" value="$(arg gui)" />
        <arg name="use_sim_time" value="true" />
    </include>
  </launch>
  ```

  _The **arg** tag is an argument parameter that can be altered to initialize different parameter values for the launch file._

- This will execute a default launch file provided by _Gazebo_, load our world file and display the _Gazebo_ client. You can launch it by doing:
  - `roslaunch navigation_robot world.launch`

## Build your own model

The more accurate you want your model to be, the more time you would need to spend on the design. The most standard way to go about creating one is placing a **SDF** file in the `~/.gazebo/models` directory, however, we will be using our newly acquired knowledge on **URDF** in tandem with **Xacro** (XML Macro) to make shorter and clearer ROS descriptions.

```bash
cd ~/sim_ws/src/navigation_robot
mkdir urdf
cd urdf
```

Now, lets build a 4-wheel robot!

- We will begin by building a blueprint for the wheels and chassis of the robot, and for that we will need a `.xacro` file to keep track of the macros!

  ```bash
  touch macros.xacro
  ```

- Paste the following in `macros.xacro`:

  ```xml
  <?xml version="1.0"?>
  <robot name="macros" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!--All units in m-kg-s-radians unit system -->
    <xacro:property name="M_PI" value="3.1415926535897931" />
    <xacro:property name="footprint_vertical_offset" value="-0.05" />

    <!-- Chassis -->
    <xacro:property name="chassis_height" value="0.1" />
    <xacro:property name="chassis_width" value="0.5" />
    <xacro:property name="chassis_length" value="0.6" />
    <xacro:property name="chassis_mass" value="5" /> <!-- in kg-->

    <!-- Wheels -->
    <xacro:property name="wheel_horizontal_separation" value="0.15" />
    <xacro:property name="wheel_vertical_separation" value="0.23" />
    <xacro:property name="wheel_vertical_offset" value="-0.13" />
    <xacro:property name="wheel_radius" value="0.098" />
    <xacro:property name="wheel_width" value="0.040" />

    <xacro:macro name="wheel" params="prefix trans_x trans_y trans_z">
      <link name="${prefix}_wheel">
        <visual>
          <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
          <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
          </geometry>
          <material name="Red" />
        </visual>
        <collision>
          <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
          <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
          </geometry>
        </collision>
        <inertial>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <mass value="0.477"/>
          <inertia
            ixx="0.0013" ixy="0" ixz="0"
            iyy="0.0024" iyz="0"
            izz="0.0013"/>
          </inertial>
      </link>

      <gazebo reference="${prefix}_wheel">
        <material>Gazebo/DarkGrey</material>
      </gazebo>

      <joint name="${prefix}_joint" type="continuous">
        <parent link="chassis_link"/>
        <child link="${prefix}_wheel" />
        <origin xyz="${trans_x} ${trans_y} ${trans_z}" rpy="0 0 0" />
        <axis xyz="0 1 0" />
      </joint>
    </xacro:macro>

    <xacro:macro name="cylinder_inertia" params="mass r l">
      <inertia  ixx="${mass*(3*r*r+l*l)/12}" ixy = "0" ixz = "0"
                iyy="${mass*(3*r*r+l*l)/12}" iyz = "0"
                izz="${mass*(r*r)/2}" />
    </xacro:macro>

  </robot>
  ```

  - the term **xacro:property** defines a named value that can serve as a variable for **XML** documents.
  - the term **xacro:macro** creates a unique named macro (XML tag) for a certain block of code, is similar to creating an object/class in programming (defining parameters and attributes of the object).
  - the term **link** describes an element for ROS to create, often including properties of inertia, visuals and collision.
  - the term **joint** describes the physical relation between links.
  - the term **gazebo** describes how the _Gazebo_ simulator will present the given description.
    > _more on [xacro](http://wiki.ros.org/xacro) and [URDF tags](http://wiki.ros.org/urdf/XML)._

Great! We now have a blueprint for our components and some. Let's put it together in another `.xacro` file!

```bash
touch robot.xacro
```

- Paste the following in `robot.xacro`:

  ```xml
  <?xml version="1.0"?>

  <robot name="navigation_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="$(find navigation_robot)/urdf/macros.xacro" />

    <!-- base_link is a fictitious link(frame) that is on the ground right below chassis_link origin -->
    <link name="base_link">
      <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.001 0.001 0.001" />
      </geometry>
      </visual>
    </link>

    <joint name="base_link_joint" type="fixed">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <parent link="base_link"/>
      <child link="chassis_link" />
    </joint>

    <!-- chassis_link is the centre frame that is essentially the body of the robot, serving as the main connector to all 4 wheels -->
    <link name="chassis_link">
      <visual>
        <origin xyz="0 0 ${footprint_vertical_offset}" rpy="0 0 0"/>
        <geometry>
          <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
        </geometry>
        <material name="Grey" />
      </visual>
      <collision>
        <origin xyz="0 0 ${footprint_vertical_offset}"/>
        <geometry>
          <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
        </geometry>
      </collision>
      <inertial>
      <!-- Center of mass -->
      <origin xyz="0.012  0.002 0.067" rpy="0 0 0"/>
      <mass value="16.523"/>
      <!-- Moments of inertia: ( chassis without wheels ) -->
      <inertia
        ixx="0.3136" ixy="-0.0008" ixz="0.0164"
        iyy="0.3922" iyz="-0.0009"
        izz="0.4485"/>
      </inertial>
    </link>

  <joint name="joint_sensor_laser" type="fixed">
    <origin xyz="0.15 0 0.05" rpy="0 0 0"/>
    <parent link="chassis_link"/>
    <child link="sensor_laser"/>
  </joint>

  <!-- laserscan! -->
  <link name="sensor_laser">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <xacro:cylinder_inertia mass="1" r="0.05" l="0.1" />
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="white" />
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <xacro:wheel prefix="front_left" trans_x="${wheel_horizontal_separation}" trans_y="${wheel_vertical_separation+wheel_width/2}" trans_z="${wheel_vertical_offset}"/>
    <xacro:wheel prefix="front_right" trans_x="${wheel_horizontal_separation}" trans_y="${-wheel_vertical_separation-wheel_width/2}" trans_z="${wheel_vertical_offset}"/>
    <xacro:wheel prefix="rear_left" trans_x="${-wheel_horizontal_separation}" trans_y="${wheel_vertical_separation+wheel_width/2}" trans_z="${wheel_vertical_offset}"/>
    <xacro:wheel prefix="rear_right" trans_x="${-wheel_horizontal_separation}" trans_y="${-wheel_vertical_separation-wheel_width/2}" trans_z="${wheel_vertical_offset}"/>

  </robot>
  ```

  > here we call on our macros from `macros.xacro` using `$()`, to help define our wheels and properties of the chassis in a concise manner!

Now we have the description of our robot! All that remains is our `.gazebo` file, which will provide _Gazebo_ with the properties we want our robot to have in the simulation!

```bash
touch robot.gazebo
```

- Paste the following in `robot.gazebo`:

  ```xml
  <?xml version="1.0"?>
  <robot>

    <gazebo reference="base_link">
      <turnGravityOff>false</turnGravityOff>
    </gazebo>

    <gazebo reference="chassis_link">
      <material>Gazebo/White</material>
      <turnGravityOff>false</turnGravityOff>
    </gazebo>

    <gazebo reference="sensor_laser">
      <material>Gazebo/Blue</material>
      <sensor type="ray" name="head_hokuyo_sensor">
        <pose>0 0 0 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>20</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>720</samples>
              <resolution>1</resolution>
              <min_angle>-1.570796</min_angle>
              <max_angle>1.570796</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.10</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
          <topicName>/navigation_robot/laser/scan</topicName>
          <frameName>sensor_laser</frameName>
        </plugin>
      </sensor>
    </gazebo>

    <gazebo>
      <plugin name="object_controller" filename="libgazebo_ros_planar_move.so">
        <commandTopic>cmd_vel</commandTopic>
        <odometryTopic>odom</odometryTopic>
        <odometryFrame>odom</odometryFrame>
        <odometryRate>20.0</odometryRate>
        <robotBaseFrame>base_link</robotBaseFrame>
      </plugin>
    </gazebo>

  </robot>
  ```

  - the term **sensor** describes a physical [sensor](http://gazebosim.org/tutorials?cat=sensors) that is used to read laserscans into the robot, providing information on the environment.
  - the term **plugin** provides functionality for the robot, connecting sensor input & messages to motor output! Read more on it [here](http://gazebosim.org/tutorials?tut=ros_gzplugins).

Now, lets create a launch file to see your very own robot in action!

```bash
cd ~/sim_ws/src/navigation_robot/launch
```

- Paste the following into `world.launch`:

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>

  <launch>
      <arg name="debug" default="false" />
      <arg name="gui" default="true" />
      <arg name="pause" default="false" />
      <arg name="world" default="$(find navigation_robot)/world/empty_world.world" />

      <!-- include gazebo_ros launcher -->
      <include file="$(find gazebo_ros)/launch/empty_world.launch">
          <arg name="world_name" value="$(arg world)" />
          <arg name="debug" value="$(arg debug)" />
          <arg name="gui" value="$(arg gui)" />
          <arg name="paused" value="$(arg pause)" />
          <arg name="use_sim_time" value="true" />
      </include>

      <!-- set params -->
      <param name="robot_description" command="$(find xacro)/xacro --inorder $(find navigation_robot)/urdf/robot.xacro" />

      <!-- rviz state values -->
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

      <!-- rviz joint values -->
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
          <param name="use_gui" value="False"/>
      </node>

      <!-- rviz -->
      <node name="rviz" pkg="rviz" type="rviz" args="-d $(find navigation_robot)/launch/config.rviz"/>

      <!-- spawn in gazebo -->
      <node name="robot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
          args="-urdf -param robot_description -model iris_model -x 0 -y 0 -z 0.5" />

  </launch>
  ```

  - **robot_description** loads the robot xacro into _Rviz_ and _Gazebo_.
  - **x_state_publisher** stabilizes the links and joints of the robot in ROS using [transformers](http://wiki.ros.org/tf).

Its time, launch your work!

```bash
roslaunch navigation_robot world.launch
```

![custom_robot_rviz](images/custom_robot_rviz.png)
![custom_robot_gazebo](images/custom_robot_gazebo.png)

---

<br>

With the current knowledge of ROS and Gazebo at your disposal, localization is within your grasp! Being able to send goals and automate path building is one of many amazing things we want a robot to be able to do. So, lets try it now!

## Launch in Docker

For the localization simulation, its available in docker also!

- Make sure you have [docker](https://docs.docker.com/engine/install/) and [docker-compose](https://docs.docker.com/compose/install/) on your OS.
  - `docker -v`
  - `docker-compose --version`
- Open a new terminal. Clone the repo below:
  ```bash
  cd ~
  git clone https://github.com/dlakhiani/iris_model.git
  cd iris_model
  xhost local:docker
  docker-compose build
  docker-compose up ros-develop-amcl
  ```
  > _Please do be patient when loading Gazebo, as it will take a bit of time due to it being a graphical client._
- Now you can send it `Goals` using Rviz, and the robot will configure a path to get to the given destination goal!

## Navigating through environment
