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
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_rgbd': False,           # Enable RGB-D (camera) subscription
            'subscribe_scan_cloud': True,     # Enable Lidar subscription
            'subscribe_imu': False,           # Set to True if you want to include IMU data
            'approx_sync': True,              # Enables approximate synchronization for Lidar and camera
            'queue_size': 100,                 # Adjust based on data rate and frequency
            'wait_for_transform': 0.2,
            'use_sim_time': use_sim_time,
            # RTAB-Map's internal parameters
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/CreateOccupancyGrid': 'false',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '30',
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.2'
        }],
        remappings=[
            # Lidar Topic
            ('scan_cloud', '/velodyne_points'),
        ],
        arguments=[
            '-d'  # Deletes the previous database (~/.ros/rtabmap.db)
        ]
    ),     
    ])

