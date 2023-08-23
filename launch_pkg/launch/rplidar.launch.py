from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'rplidar_ros', 'rplidar_composition',
                 '--ros-args', '-p', 'serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                 '-p', 'frame_id:=laser_frame',
                 '-p', 'angle_compensate:=true',
                 '-p', 'scan_mode:=Standard', '-p', 'use_intra_process_comms:=true'],
            output='screen' 
        )
    ])
