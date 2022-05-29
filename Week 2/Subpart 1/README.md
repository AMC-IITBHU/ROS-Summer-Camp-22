<details>
  <summary><h1>Gazebo </h1></summary>
  
  Gazebo is an open-source 3D robotics simulator. With Gazebo you are able to create a 3D scenario on your computer with robots, obstacles and many other objects. Gazebo also uses a physical engine for illumination, gravity, inertia, etc. You can evaluate and test your robot in difficult or dangerous scenarios without any harm to your robot. Most of the time it is faster to run a simulator instead of starting the whole scenario on your real robot.
  
  <p align="center">
    <img width=500 src="https://github.com/AMC-IITBHU/ROS-Summer-Camp-22/blob/WEEK2BRANCH/Week%202/assests/images%20(1).jpeg">
  </p>
  
  So let's start learning what gazebo really is :-
  
  # Part 1 - 
  
  Action first before the theory! Let's create a project in gazebo. Create a new package in your workspace with name of your choice. For now I am using the name "gazebo_tutorials". Inside the package create a folder world. Create a new file inside the folder with extension .world and place the following content (just do copy paste)
  
  ```xml
  <sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name='unit_box'>
      <pose>-1.53411 -1.50222 0.5 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <model name='Mailbox'>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://mailbox/meshes/mailbox.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://mailbox/meshes/mailbox.dae</uri>
            </mesh>
          </geometry>
          <material>
            <script>
              <uri>model://mailbox/materials/scripts</uri>
              <uri>model://mailbox/materials/textures</uri>
              <name>Mailbox/Diffuse</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
          <mass>1</mass>
        </inertial>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>0.730733 2.14938 0 0 -0 0</pose>
    </model>
    <model name='table_marble'>
      <static>1</static>
      <pose>2.89095 -1.16521 0.648 0 -0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model:///table_marble/meshes/table_lightmap.dae</uri>
              <scale>0.25 0.25 0.25</scale>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://table_marble/meshes/table_lightmap.dae</uri>
              <scale>0.25 0.25 0.25</scale>
            </mesh>
          </geometry>
          <material>
            <script>
              <uri>model://table_marble/materials/scripts</uri>
              <uri>model://table_marble/materials/textures</uri>
              <name>Table/Marble_Lightmap</name>
            </script>
            <lighting>0</lighting>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0.120272 -0.218971 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='cafe_table'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose>0 0 0.755 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.913 0.913 0.04</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <collision name='column'>
          <pose>0 0 0.37 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.042 0.042 0.74</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <collision name='base'>
          <pose>0 0 0.02 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.56 0.56 0.04</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://cafe_table/meshes/cafe_table.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>4.93489 1.74308 0 0 -0 0</pose>
    </model>
    <model name='first_2015_trash_can'>
      <link name='link'>
        <inertial>
          <pose>0 0 0.3683 0 -0 0</pose>
          <mass>4.83076</mass>
          <inertia>
            <ixx>0.281534</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.281534</iyy>
            <iyz>0</iyz>
            <izz>0.126223</izz>
          </inertia>
        </inertial>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://first_2015_trash_can/meshes/trash_can.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://first_2015_trash_can/meshes/trash_can.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>4.52055 -4.82605 0 0 -0 0</pose>
    </model>
    <state world_name='default'>
      <sim_time>106 649000000</sim_time>
      <real_time>107 71423633</real_time>
      <wall_time>1653808482 782998527</wall_time>
      <iterations>106649</iterations>
      <model name='Mailbox'>
        <pose>0.730733 2.14938 -9e-06 -9e-06 1e-06 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.730733 2.14938 -9e-06 -9e-06 1e-06 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='cafe_table'>
        <pose>4.93489 1.74308 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>4.93489 1.74308 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='first_2015_trash_can'>
        <pose>4.52057 -4.82606 1e-06 7e-06 -6e-06 -9.1e-05</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>4.52057 -4.82606 1e-06 7e-06 -6e-06 -9.1e-05</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>5.42105 -0.77753 3.13165 -1.03069 0.989248 3.14157</acceleration>
          <wrench>26.1878 -3.75606 15.1283 0 -0 0</wrench>
        </link>
      </model>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='table_marble'>
        <pose>2.89095 -1.16521 0.648 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>2.89095 -1.16521 0.648 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box'>
        <pose>-1.53411 -1.50222 0.499995 0 1e-05 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-1.53411 -1.50222 0.499995 0 1e-05 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0.010615 -0.006191 -9.78231 0.012424 0.021225 1.8e-05</acceleration>
          <wrench>0.010615 -0.006191 -9.78231 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-1.1263 -18.8954 54.7285 -0 1.2578 1.0602</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
```

  Let the full name of file be new.world
  
  Now open a terminal and navigate to the world folder that we created and type
  
  ```bash
  gazebo new.world
  ```
  
  And you will see a new window of gazebo with some models in it.
  
  # Part 2 :
  
  Now we need to learn how to launch gazebo from a launch file. You definately don't want to open gazebo everytime by opening a terminal and then navigating to the folder containing launch file and blah blah. You need to learn to open gazebo from a launch file. 
  
  
  
</details>
