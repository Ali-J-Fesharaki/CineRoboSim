#!/usr/bin/env python3
"""
Gazebo Camera Orbit ROS2 Node
-----------------------------
ROS2 node that controls Ignition Gazebo camera via services,
orbits around a center point, captures frames, and creates animated GIFs.

Prerequisites: Gazebo must already be running!

Parameters:
    center_x, center_y, center_z: Center point to orbit around (default: 0, 0, 0)
    radius: Orbit radius in meters (default: 20.0)
    height: Camera height above center_z (default: 15.0)
    frames: Number of frames for orbit (default: 30)
    output_file: Output GIF filename (default: output.gif)

Usage:
    ros2 run sdf_to_gif sdf_to_gif_node --ros-args -p center_x:=5.0 -p center_y:=5.0 -p radius:=25.0
"""

import os
import time
import subprocess
import math
import shutil
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SdfToGifNode(Node):
    """ROS2 Node for orbiting camera and capturing GIFs from Ignition Gazebo"""
    
    def __init__(self):
        super().__init__('sdf_to_gif_node')
        
        # Declare parameters - center point
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_z', 0.0)
        
        # Declare parameters - orbit
        self.declare_parameter('radius', 20.0)
        self.declare_parameter('height', 15.0)
        self.declare_parameter('frames', 30)
        
        # Declare parameters - output
        self.declare_parameter('output_file', 'output.gif')
        self.declare_parameter('output_dir', 'gifs')
        
        # Declare parameters - misc
        self.declare_parameter('cmd', 'ign')  # 'ign' or 'gz'
        self.declare_parameter('settle_time', 0.3)  # Time between frames
        self.declare_parameter('start_angle', 0.0)  # Starting angle in radians
        self.declare_parameter('gif_duration', 3.0)  # Total GIF duration in seconds
        
        # Get parameters - center point
        self.center_x = self.get_parameter('center_x').get_parameter_value().double_value
        self.center_y = self.get_parameter('center_y').get_parameter_value().double_value
        self.center_z = self.get_parameter('center_z').get_parameter_value().double_value
        
        # Get parameters - orbit
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.height = self.get_parameter('height').get_parameter_value().double_value
        self.frames = self.get_parameter('frames').get_parameter_value().integer_value
        
        # Get parameters - output
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        
        # Get parameters - misc
        self.cmd = self.get_parameter('cmd').get_parameter_value().string_value
        self.settle_time = self.get_parameter('settle_time').get_parameter_value().double_value
        self.start_angle = self.get_parameter('start_angle').get_parameter_value().double_value
        self.gif_duration = self.get_parameter('gif_duration').get_parameter_value().double_value
        
        # Publisher for status
        self.status_pub = self.create_publisher(String, 'sdf_to_gif/status', 10)
        
        self.get_logger().info('Gazebo Camera Orbit Node initialized')
        self.get_logger().info(f'  Center: ({self.center_x}, {self.center_y}, {self.center_z})')
        self.get_logger().info(f'  Radius: {self.radius}m, Height: {self.height}m')
        self.get_logger().info(f'  Frames: {self.frames}')
        self.get_logger().info(f'  Output: {self.output_dir}/{self.output_file}')
        
        # Start processing in a timer callback (non-blocking)
        self.create_timer(1.0, self.process_once)
        self._processed = False
    
    def process_once(self):
        """Process once, then shutdown"""
        if self._processed:
            return
        self._processed = True
        
        try:
            # Check if Gazebo services are available
            if not self.wait_for_service('/gui/screenshot', timeout=10):
                self.get_logger().error('Gazebo GUI not running! Start Gazebo first.')
                rclpy.shutdown()
                return
            
            self.capture_gif()
        except Exception as e:
            self.get_logger().error(f'Error during processing: {e}')
        finally:
            self.get_logger().info('Processing complete. Shutting down...')
            rclpy.shutdown()
    
    def publish_status(self, status: str):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)
    
    # =========================================================================
    # GAZEBO SERVICE FUNCTIONS
    # =========================================================================
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to Quaternion"""
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return {'x': qx, 'y': qy, 'z': qz, 'w': qw}
    
    def calculate_lookat_quaternion(self, eye_x, eye_y, eye_z, target_x, target_y, target_z):
        """Calculate quaternion to point camera at target"""
        dx = target_x - eye_x
        dy = target_y - eye_y
        dz = target_z - eye_z
        
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if dist == 0:
            return {'x': 0, 'y': 0, 'z': 0, 'w': 1}
        
        fx = dx / dist
        fy = dy / dist
        fz = dz / dist
        
        pitch = -math.asin(np.clip(fz, -1.0, 1.0))
        yaw = math.atan2(fy, fx)
        roll = 0.0
        
        return self.euler_to_quaternion(roll, pitch, yaw)
    
    def get_default_screenshot_dir(self):
        """Get the default directory where Gazebo saves screenshots"""
        home = os.path.expanduser("~")
        return [
            os.path.join(home, ".ignition", "gui", "pictures"),
            os.path.join(home, ".gz", "gui", "pictures")
        ]
    
    def take_screenshot(self):
        """Trigger internal screenshot service"""
        msg_pkg = "ignition.msgs" if "ign" in self.cmd else "gz.msgs"
        
        cmd_args = [
            self.cmd, 'service', '-s', '/gui/screenshot',
            '--reqtype', f'{msg_pkg}.StringMsg',
            '--reptype', f'{msg_pkg}.Boolean',
            '--timeout', '2000',
            '--req', 'data: ""'
        ]
        
        result = subprocess.run(cmd_args, capture_output=True, text=True)
        
        if result.returncode != 0:
            return False
        elif "data: false" in result.stdout.lower():
            return False
        return True
    
    def move_camera_quat(self, x, y, z, qx, qy, qz, qw):
        """Move camera using quaternion orientation"""
        req_str = (
            f'pose: {{ '
            f'position: {{ x: {x}, y: {y}, z: {z} }}, '
            f'orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }} '
            f'}}'
        )
        
        cmd_args = [
            self.cmd, 'service', '-s', '/gui/move_to/pose',
            '--reqtype', 'ignition.msgs.GUICamera',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000',
            '--req', req_str
        ]
        
        result = subprocess.run(cmd_args, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().warn(f'Camera move failed')
    
    def wait_for_service(self, service_name: str, timeout=30):
        """Wait for a service to become available"""
        self.get_logger().info(f'Waiting for service {service_name}...')
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                output = subprocess.check_output(
                    [self.cmd, 'service', '-l'], stderr=subprocess.DEVNULL
                ).decode()
                if service_name in output:
                    self.get_logger().info(f'Service {service_name} is available.')
                    return True
            except Exception:
                pass
            time.sleep(1)
        self.get_logger().error(f'Timeout waiting for {service_name}')
        return False
    
    # =========================================================================
    # ANIMATION FUNCTIONS
    # =========================================================================
    
    def capture_single_frame(self, frame_index: int, frames_dir: str):
        """Capture a single screenshot and save it"""
        frame_path = os.path.join(frames_dir, f"frame_{frame_index:03d}.png")
        
        search_dirs = self.get_default_screenshot_dir()
        files_before = {}
        for d in search_dirs:
            if os.path.exists(d):
                try:
                    files_before[d] = set(os.listdir(d))
                except:
                    files_before[d] = set()
        
        success = self.take_screenshot()
        
        if not success:
            return None
        
        timeout = 5.0
        start_wait = time.time()
        
        while (time.time() - start_wait) < timeout:
            for d in search_dirs:
                if not os.path.exists(d):
                    continue
                try:
                    files_now = set(os.listdir(d))
                    new_files = files_now - files_before.get(d, set())
                    
                    for nf in new_files:
                        if nf.endswith('.png'):
                            fp = os.path.join(d, nf)
                            shutil.move(fp, frame_path)
                            return frame_path
                except:
                    pass
            time.sleep(0.2)
        
        return None
    
    def animate_orbit(self, focal_x, focal_y, focal_z,
                      orbit_radius, orbit_height,
                      start_angle, num_frames, frames_dir,
                      frame_offset=0, settle_time=0.3):
        """Perform orbit animation"""
        captured = []
        
        self.get_logger().info(f'Orbit: {num_frames} frames, radius={orbit_radius:.1f}m')
        
        for i in range(num_frames):
            progress = i / num_frames
            theta = start_angle + (2 * math.pi * progress)
            
            cam_x = focal_x + orbit_radius * math.cos(theta)
            cam_y = focal_y + orbit_radius * math.sin(theta)
            cam_z = orbit_height
            
            q = self.calculate_lookat_quaternion(cam_x, cam_y, cam_z, focal_x, focal_y, focal_z)
            
            self.move_camera_quat(cam_x, cam_y, cam_z, q['x'], q['y'], q['z'], q['w'])
            time.sleep(settle_time)
            
            frame_idx = frame_offset + len(captured)
            frame_path = self.capture_single_frame(frame_idx, frames_dir)
            if frame_path:
                captured.append(frame_path)
        
        return captured
    
    # =========================================================================
    # MAIN CAPTURE FUNCTION
    # =========================================================================
    
    def capture_gif(self):
        """Orbit camera around center point and create GIF"""
        
        # Ensure output directory exists
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        output_path = os.path.join(self.output_dir, self.output_file)
        
        # Create frames directory
        frames_dir = os.path.abspath(os.path.join("frames", "capture"))
        if os.path.exists(frames_dir):
            shutil.rmtree(frames_dir)
        os.makedirs(frames_dir)
        
        # Orbit around center point
        self.publish_status(f'Orbiting around ({self.center_x}, {self.center_y}, {self.center_z}) with radius {self.radius}m')
        
        captured = self.animate_orbit(
            self.center_x, self.center_y, self.center_z,
            self.radius, self.center_z + self.height,
            self.start_angle, self.frames, frames_dir,
            frame_offset=0,
            settle_time=self.settle_time
        )
        
        # Generate GIF
        self.publish_status('Generating GIF...')
        
        if not captured:
            self.publish_status('Error: No frames captured!')
            return
        
        self.get_logger().info(f'Total frames: {len(captured)}')
        
        # Create GIF - calculate per-frame duration from total duration
        frame_duration_ms = int((self.gif_duration * 1000) / len(captured))
        self.get_logger().info(f'GIF: {self.gif_duration}s total, {frame_duration_ms}ms per frame')
        
        images = [Image.open(f) for f in captured]
        images[0].save(
            output_path,
            save_all=True,
            append_images=images[1:],
            duration=frame_duration_ms,
            loop=0
        )
        
        self.publish_status(f'Saved {output_path}')


def main(args=None):
    rclpy.init(args=args)
    node = SdfToGifNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
