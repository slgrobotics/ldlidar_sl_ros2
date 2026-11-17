**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki)

------------------

**Note:**
- This fork implements *"constant beam count"* feature - ROS2 fails without it.
- STL-19P (LD19P) LIDAR has 230400 baud rate and ```'product_name': 'LDLiDAR_LD14P'``` setting works for it.
- See https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md for setup instructions.
- See [this file](https://github.com/slgrobotics/articubot_one/blob/lyrical/robots/seggy/launch/seggy.sensors.launch.py) for an example of launch.
- The driver reads parameters at startup, so restart the node to apply changes.

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `product_name` | string | `""` | Selects `LDLiDAR_LD14` or `LDLiDAR_LD14P`. Seggy’s FHL-LD19 launch uses `LDLiDAR_LD14P`. |
| `port_name` | string | `""` | Serial device path. An empty value causes startup to fail. See [this](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/PersistentUSB.md) |
| `serial_baudrate` | integer | `0` | Serial baud rate. Seggy’s LD19 launch sets `230400`. |
| `limit_scan_rate` | double | `6.0` | Set it to `0.0` to publish each available complete revolution at the sensor's rotation rate (~10 Hz). |
| `laser_scan_topic_name` | string | `""` | Output topic for `sensor_msgs/msg/LaserScan`. |
| `point_cloud_2d_topic_name` | string | `""` | Output topic for the 2D `sensor_msgs/msg/PointCloud`. |
| `frame_id` | string | `"base_laser"` | Frame ID stamped on the output messages. |
| `laser_scan_dir` | boolean | `true` | `true` for counterclockwise, `false` for clockwise. |
| `enable_angle_crop_func` | boolean | `false` | Enables masking a sector of the scan. |
| `angle_crop_min` | double | `0.0` | Lower bound of the masked sector, in degrees. |
| `angle_crop_max` | double | `0.0` | Upper bound of the masked sector, in degrees. |
| `min_intensity` | integer | `0` | Filters out weak returns. |
| `do_filtering` | boolean | `true` | Enables the SDK’s `NearFilter` processing. |
| `do_triplets` | boolean | `false` | Combines beams in groups of three, reducing the beam count by about a factor of three. |

The scan range bounds are hard-coded as `LIDAR_RANGE_MIN = 0.1` m and `LIDAR_RANGE_MAX = 12.0` m. Standard ROS node parameters such as `use_sim_time` are separate from these driver settings.

### Scan publication rate

Set the double parameter `limit_scan_rate` at startup to cap scan publication
in Hz. The default is `6.0`. Set it to `0.0` to publish each available complete
revolution at the sensor's rotation rate. The setting limits the driver's polling
and publication loop. It does not change the motor speed. Restart the driver
after changing it.

```bash
ros2 run ldlidar_sl_ros2 ldlidar_sl_ros2_node --ros-args   --params-file /path/to/lidar.yaml -p limit_scan_rate:=0.0
```

Values must be finite and nonnegative.

================================= Original README: ======================================

# Instructions

> This SDK is only applicable to the LiDAR products sold by Shenzhen LDROBOT Co., LTD. The product models are :
> - LDROBOT LiDAR LD14
> - LDROBOT LiDAR LD14P

## step 0: get LiDAR ROS2 Package
```bash
cd ~

mkdir -p ldlidar_ros2_ws/src

cd ldlidar_ros2_ws/src

git clone  https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
```
## step 1: system setup
- Connect the LiDAR to your system motherboard via an onboard serial port or usB-to-serial module (for example, CP2102 module).

- Set the -x permission for the serial port device mounted by the radar in the system (for example, /dev/ttyUSB0)

  - In actual use, the LiDAR can be set according to the actual mounted status of your system, you can use 'ls -l /dev' command to view.

``` bash
cd ~/ldlidar_ros2_ws

sudo chmod 777 /dev/ttyUSB0
```
- Modify the `port_name` value in the Lanuch file corresponding to the radar product model under `launch/`, using `ld14.launch.py` as an example, as shown below.

```py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar_sl_ros2',
      executable='ldlidar_sl_ros2_node',
      name='ldlidar_publisher_ld14',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'serial_baudrate' : 115200},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0},
        {'min_intensity': 0},
        {'do_filtering': True},
        {'do_triplets': False}
      ]
  )

  # base_link to base_laser tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld14',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld
```

## step 2: build

Run the following command.

```bash
cd ~/ldlidar_ros2_ws

colcon build
```
## step 3: run

### step3.1: package environment variable settings

- After the compilation is completed, you need to add the relevant files generated by the compilation to the environment variables, so that the ROS environment can recognize them. The execution command is as follows. This command is to temporarily add environment variables to the terminal, which means that if you reopen a new terminal, you also need to re-execute it. The following command.
  
  ```bash
  cd ~/ldlidar_ros2_ws
  
  source install/local_setup.bash
  ```
  
- In order to never need to execute the above command to add environment variables after reopening the terminal, you can do the following.

  ```bash
  echo "source ~/ldlidar_ros2_ws/install/local_setup.bash" >> ~/.bashrc
  
  source ~/.bashrc
  ```
### step3.2: start LiDAR node

- The product is LDROBOT LiDAR LD14
  - start ld14 lidar node:
  ``` bash
  ros2 launch ldlidar_sl_ros2 ld14.launch.py
  ```
  - start ld14 lidar node and show on the Rviz2:
  ``` bash
  ros2 launch ldlidar_sl_ros2 viewer_ld14.launch.py
  ```

- The product is LDROBOT LiDAR LD14P
  - start ld14p lidar node:
  ``` bash
  ros2 launch ldlidar_sl_ros2 ld14p.launch.py
  ```
  - start ld14p lidar node and show on the Rviz2:
  ``` bash
  ros2 launch ldlidar_sl_ros2 viewer_ld14p.launch.py
  ```

## step 4: test

> The code supports ubuntu 20.04 ros2 foxy version and above, using rviz2 visualization.

- new a terminal (Ctrl + Alt + T) and use Rviz2 tool(run command: `rviz2`) ,open the `ldlidar.rviz` file below the rviz2 folder of the readme file directory
```bash
rviz2
```
