from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

  # args that can be set from the command line or a default will be used
  rm = DeclareLaunchArgument(
      "rm1", default_value="rm1"
  )

  return LaunchDescription([
    launch_ros.actions.Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=[os.path.join(get_package_share_directory("pose_uwb"), 'params_filter', 'param_rm.yaml')],
      remappings=[('/odometry/filtered', rm.name + '/filtered'),
                  ('/set_pose', rm.name + '/set_pose')]
    ),
  ])