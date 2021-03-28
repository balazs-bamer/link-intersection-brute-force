#!/usr/bin/env python3
import os
import sys
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
def generate_launch_description():
    # Load the URDF into a parameter
    bringup_dir = get_package_share_directory('link-intersection-brute-force')
    urdf_path = os.path.join(bringup_dir, 'urdf', 'px150_coll.urdf')
    urdf = open(urdf_path).read()
  )
    return launch.LaunchDescription([),
        launch_ros.actions.Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
        ),
       ),
     )
    ])
def main(argv=sys.argv[1:]):
    """Run lifecycle nodes via launch."""
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()
if __name__ == '__main__':
    main()

# https://docs.ros.org/en/foxy/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher.html
# KELL majd world link URDFbe
