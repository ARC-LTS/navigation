import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')
    
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'deskewing', default_value='true',
            description='Enable lidar deskewing'),
          
		# Static Transform Publisher for VLP16 Lidar
		Node(
        	package='tf2_ros',
       	 	executable='static_transform_publisher',
        	name='base_to_velodyne_broadcaster',
        	output='screen',
        	arguments=['-0.25', '0', '0.1', '0', '0', '0', 'base_link', 'velodyne']
    	),

        Node(
             package='rtabmap_odom', executable='icp_odometry', output='screen',
             parameters=[{
               'frame_id':'base_link',
               'odom_frame_id':'odom',
               'publish_tf': True, 
               'wait_for_transform':0.4,
               'expected_update_rate':60.0,
               'use_sim_time':use_sim_time,
             }],
             remappings=[
               ('scan_cloud', '/sensors/lidar3d_0/points'),
               #('imu', '/sensors/imu_0/data'),
               ('odom', '/icp/odom'),
             ],
             arguments=[
               'Icp/PointToPlane', 'true',
               'Icp/Iterations', '10',
               'Icp/VoxelSize', '0.1',
               'Icp/Epsilon', '0.001',
               'Icp/PointToPlaneK', '20',
               'Icp/PointToPlaneRadius', '0',
               'Icp/MaxTranslation', '2',
               'Icp/MaxCorrespondenceDistance', '1',
               'Icp/Strategy', '1',
               'Icp/OutlierRatio', '0.7',
               'Icp/CorrespondenceRatio', '0.01',
               'Odom/ScanKeyFrameThr', '0.4',
               'Odom/ResetCountdown', '1',
               'OdomF2M/ScanSubtractRadius', '0.1',
               'OdomF2M/ScanMaxSize', '60000',
               'OdomF2M/BundleAdjustment', 'false',
             ]),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
              'frame_id':'base_link',
              'odom_frame_id':'odom',
              'subscribe_depth': False,
              'subscribe_rgb': False,
              'subscribe_scan': False,
              'subscribe_scan_cloud':True,
              'subscribe_odom_info':False,
              'approx_sync': True,
              'wait_for_transform':0.4,
              'use_sim_time': use_sim_time,
              'publish_tf': True,  # Disable publishing TF
              'queue_size': 100,
              # 'map_always_update': True,
            }],
            remappings=[
              ('scan_cloud','/sensors/lidar3d_0/points'),
            #  ('rgb/image', '/camera/color/image_raw'),
            #  ('rgb/camera_info', '/camera/color/camera_info'),
            #  ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
              ('odom', '/icp/odom'),  # Use filtered odom
              ],
            arguments=[
             '-d', # This will delete the previous database (~/.ros/rtabmap.db)
              'RGBD/ProximityMaxGraphDepth', '0',
              'RGBD/ProximityPathMaxNeighbors', '1',
              'RGBD/AngularUpdate', '0.05',
              'RGBD/LinearUpdate', '0.05',
              'RGBD/CreateOccupancyGrid', 'false',
              'Mem/NotLinkedNodesKept', 'true',
              'Mem/STMSize', '30',
              'Mem/LaserScanNormalK', '20',
              'Reg/Strategy', '1',
              'Icp/VoxelSize', '0.1',
              'Icp/PointToPlaneK', '20',
              'Icp/PointToPlaneRadius', '0',
              'Icp/PointToPlane', 'true',
              'Icp/Iterations', '10',
              'Icp/Epsilon', '0.001',
              'Icp/MaxTranslation', '3',
              'Icp/MaxCorrespondenceDistance', '1',
              'Icp/Strategy', '1',
              'Icp/OutlierRatio', '0.7',
              'Icp/CorrespondenceRatio', '0.2',
              'Grid/GroundIsObstacle', 'false',
              'Grid/MaxGroundHeight', '0.2',
              'Grid/MinObstacleHeight', '0.2',
              'Grid/NormalsSegmentation', 'true',
              'Grid/MaxObstacleHeight', '10',
              'Grid/RangeMax', '6.0',
              'Grid/RayTracing', 'True'
            ]),
        ])

