import os
import time
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def rename_rtabmap_db():
    """Check for rtabmap.db in /root/.ros/, and if it exists, rename it."""
    db_path = "/root/.ros/rtabmap.db"
    if os.path.exists(db_path):
        # Generate a timestamped filename
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        new_db_path = f"/root/.ros/rtabmap_{timestamp}.db"
        os.rename(db_path, new_db_path)
        print(f"Renamed {db_path} to {new_db_path}")
    else:
        print("No rtabmap.db file found. No action taken.")

def generate_launch_description():
    rename_rtabmap_db()
    use_sim_time = LaunchConfiguration('use_sim_time')
    output = LaunchConfiguration('output')
    
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'output', default_value='log',
            description='Output destination for nodes (screen or log)'),

#        Node(
#            package='tf2_ros',
#            executable='static_transform_publisher',
#            arguments = ['0.1016', '0', '0.500', '0', '0', '0', 'base_link', 'laser_sensor_frame']
#        ),

        # Nodes to launch
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([os.path.join(
#                get_package_share_directory('velodyne_driver'), 'launch'),
#                '/velodyne_driver_node-VLP16-launch.py']),
#        ),
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([os.path.join(
#                get_package_share_directory('velodyne_pointcloud'), 'launch'),
#                '/velodyne_transform_node-VLP16-launch.py']),
#        ),

#		# Static Transform Publisher for VLP16 Lidar
#		Node(
#        	package='tf2_ros',
#       	 	executable='static_transform_publisher',
#        	name='base_to_velodyne_broadcaster',
#        	output=output,
#        	arguments=['-0.25', '0', '0.1', '0', '0', '0', 'base_link', 'velodyne']
#    	),

         # Nodes to launch
        TimerAction(period=5.0,actions=[

           Node(
             package='rtabmap_odom', executable='icp_odometry', output=output,
             parameters=[{
               'frame_id':'base_link',
               'odom_frame_id':'odom',
               'publish_tf': True,
               'wait_for_transform':2.0,  # wall-clock wait for sim-time TF; 0.4 starves under low RTF and stalls scan conversion
               'expected_update_rate':10.0,   # VLP16 default rotation rate
               'use_sim_time':use_sim_time,
             }],
             remappings=[
               ('scan_cloud', '/sensors/lidar3d_0/points'),
               #('imu', '/imu'),
               #('odom', '/icp/odometry'),
             ],
             arguments=[
               'Icp/PointToPlane', 'true',
               'Icp/Iterations', '10',
               'Icp/VoxelSize', '0.15',        # VLP16 ring density normalisation
               'Icp/Epsilon', '0.001',
               'Icp/PointToPlaneK', '20',
               'Icp/PointToPlaneRadius', '0',
               'Icp/MaxTranslation', '2',
               'Icp/MaxCorrespondenceDistance', '0.5',  # tight enough to reject cross-aisle false matches
               'Icp/Strategy', '1',
               'Icp/OutlierRatio', '0.65',
               'Icp/CorrespondenceRatio', '0.01',
               'Odom/ScanKeyFrameThr', '0.4',
               'Odom/ResetCountdown', '1',
               'OdomF2M/ScanSubtractRadius', '0.15',   # match voxel size
               'OdomF2M/ScanMaxSize', '100000',         # ~3 full VLP16 scans of reference map
               'OdomF2M/BundleAdjustment', 'false',
               '--ros-args', '--log-level', 'error'
             ]),
            
        # Node(
        #    package='rtabmap_odom', executable='rgbd_odometry', output=output,
        #    parameters=[{
        #     'frame_id':'base_link',
        #     'subscribe_depth':True,
        #     'subscribe_odom_info':True,
        #     'approx_sync':True
        #     }],
        #   remappings=[
        #     ('rgb/image', '/camera/color/image_raw'),
        #     ('rgb/camera_info', '/camera/color/camera_info'),
        #     ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        #   ]),
          
        Node(
            package='rtabmap_slam', executable='rtabmap', output=output,
            parameters=[{
              'frame_id':'base_link',
              'odom_frame_id':'odom',
              'subscribe_depth': False,
              'subscribe_rgb': False,
              'subscribe_scan': False,
              'subscribe_scan_cloud':True,
              'subscribe_odom_info':False,
              'approx_sync': True,
              'wait_for_transform':2.0,  # wall-clock wait for sim-time TF; 0.4 starves under low RTF and stalls scan conversion
              'use_sim_time': use_sim_time,
              'publish_tf': True,  # Disable publishing TF
              'queue_size': 30,
              # 'map_always_update': True,
            }],
            remappings=[
              ('scan_cloud','/sensors/lidar3d_0/points'),
            #  ('rgb/image', '/camera/color/image_raw'),
            #  ('rgb/camera_info', '/camera/color/camera_info'),
            #  ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            #  ('odom', '/odometry'),  # Use filtered odom
              ],
            arguments=[
             '-d', # This will delete the previous database (~/.ros/rtabmap.db)
              'RGBD/ProximityMaxGraphDepth', '0',
              'RGBD/ProximityPathMaxNeighbors', '1',
              'RGBD/AngularUpdate', '0.05',
              'RGBD/LinearUpdate', '0.05',
              'RGBD/CreateOccupancyGrid', 'false',
              'Mem/NotLinkedNodesKept', 'true',
              'Mem/STMSize', '50',              # more recent nodes available for loop closure
              'Mem/LaserScanNormalK', '20',
              'Reg/Strategy', '1',
              'Icp/VoxelSize', '0.15',          # match odometry voxel size
              'Icp/PointToPlaneK', '20',
              'Icp/PointToPlaneRadius', '0',
              'Icp/PointToPlane', 'true',
              'Icp/Iterations', '10',
              'Icp/Epsilon', '0.001',
              'Icp/MaxTranslation', '3',
              'Icp/MaxCorrespondenceDistance', '0.5',  # tight matching reduces misaligned scan merges
              'Icp/Strategy', '1',
              'Icp/OutlierRatio', '0.65',
              'Icp/CorrespondenceRatio', '0.3',  # require more valid correspondences before accepting scan
              'Grid/GroundIsObstacle', 'false',
              'Grid/MaxGroundHeight', '0.15',
              'Grid/MinObstacleHeight', '0.1',   # reject near-ground noise from clearing cells
              'Grid/NormalsSegmentation', 'true',
              'Grid/MaxObstacleHeight', '2.0',   # VLP16 +15° ceiling returns excluded; preserves shelf cells
              'Grid/RangeMax', '8.0',            # shorter ray-traces = less sweeping through mapped obstacles
              'Grid/RayTracing', 'true',
              'Grid/ProbMiss', '0.45',           # weaker clearing vote per ray; occupied cells survive longer
              '--ros-args', '--log-level', 'error'
            ]),
        ]),
    ])
