import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # TurtleBot3'ün gazebo dünyası için launch dosyasının yolunu bul
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_world_launch = os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')

    return LaunchDescription([
        # TurtleBot3 gazebo dünyasını başlat
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_world_launch)
        ),
	    # Çizgi takip (line detector) düğümünü başlat
        ExecuteProcess(
            cmd=['ros2', 'run', 'otonom_surus', 'line_detector_node'],
            output='screen'
        ),
        # Karar verici düğümü başlat
        ExecuteProcess(
            cmd=['ros2', 'run', 'otonom_surus', 'karar_verici'],
            output='screen'
        ),
        
        # Uyku tespit düğümünü başlat
        ExecuteProcess(
            cmd=['ros2', 'run', 'otonom_surus', 'sleep_detection'],
            output='screen'
        ),





    ])

