import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  packageName = 'link-intersection-brute-force'
  nodeName = 'linkIntersectionBruteForce'
  urdfFilename = 'px150_coll.urdf'
  urdf = os.path.join(
    get_package_share_directory(packageName),
    urdfFilename)
  forbiddenLinksFilename = 'forbidden-links.txt'
  forbiddenLinks = os.path.join(
    get_package_share_directory(packageName),
    forbiddenLinksFilename)
  return LaunchDescription([
    Node(
      package = packageName,
      namespace = 'cudaTrajectoryPlanner',
      executable = nodeName,
      name = nodeName,
      arguments = [urdf, forbiddenLinks, '/home/balazs/munka/cuda-trajectory-planner/ros-workspace/src/link-intersection-brute-force/']
    )
  ])
