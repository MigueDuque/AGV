from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'rplidar_ros', 'rplidar_composition',
                 '--ros-args', '-p', 'serial_port:=/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0',
                 '-p', 'frame_id:=laser_frame',
                 '-p', 'angle_compensate:=true',
                 '-p', 'scan_mode:=Standard', '-p', 'use_intra_process_comms:=true'],
            output='screen' 
        )
    ])
