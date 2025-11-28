#!/usr/bin/env python3
"""
SDF to Video Converter
----------------------
Iterates through SDF files in a directory, launches Ignition Gazebo,
orbits the camera around the world, and records video using Ignition's
built-in video recorder.

Usage:
    python3 sdf_to_video.py --input maps/generated --output videos
    python3 sdf_to_video.py maze.sdf --duration 5.0 --format mp4
"""

import os
import sys
import time
import subprocess
import argparse
import math
import glob
import shutil
import re
import tempfile
import numpy as np
import signal
import xml.etree.ElementTree as ET

def kill_gazebo_processes():
    """Kill all gazebo/ruby processes forcefully"""
    try:
        # Kill ruby processes (used by ign/gz)
        subprocess.run(['pkill', '-9', '-f', 'ruby.*ign'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', '-f', 'ruby.*gz'], stderr=subprocess.DEVNULL)
        # Kill gzserver and gzclient
        subprocess.run(['pkill', '-9', 'gzserver'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', 'gzclient'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', '-f', 'ign gazebo'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], stderr=subprocess.DEVNULL)
        time.sleep(1)
    except Exception as e:
        print(f"Warning: Could not kill processes: {e}")

def get_world_bounds_from_sdf(sdf_file):
    """Parse SDF file to get the bounding box of all models"""
    try:
        tree = ET.parse(sdf_file)
        root = tree.getroot()
        
        min_x, max_x = float('inf'), float('-inf')
        min_y, max_y = float('inf'), float('-inf')
        
        # Find all pose elements
        for model in root.iter('model'):
            pose_elem = model.find('pose')
            if pose_elem is not None and pose_elem.text:
                parts = pose_elem.text.strip().split()
                if len(parts) >= 2:
                    x, y = float(parts[0]), float(parts[1])
                    min_x = min(min_x, x)
                    max_x = max(max_x, x)
                    min_y = min(min_y, y)
                    max_y = max(max_y, y)
        
        if min_x == float('inf'):
            return 0, 0, 15.0  # Default center and radius
            
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        
        # Calculate appropriate radius based on world size
        world_size = max(max_x - min_x, max_y - min_y)
        radius = world_size * 0.8  # 80% of world size for good view
        radius = max(radius, 5.0)  # Minimum radius of 5m
        
        print(f"World bounds: X[{min_x:.1f}, {max_x:.1f}] Y[{min_y:.1f}, {max_y:.1f}]")
        print(f"World center: ({center_x:.1f}, {center_y:.1f}), Orbit radius: {radius:.1f}")
        
        return center_x, center_y, radius
        
    except Exception as e:
        print(f"Warning: Could not parse SDF bounds: {e}")
        return 0, 0, 15.0

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to Quaternion"""
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return {'x': qx, 'y': qy, 'z': qz, 'w': qw}

def calculate_lookat_quaternion(eye_x, eye_y, eye_z, target_x, target_y, target_z):
    """
    Calculates the quaternion to point a camera at a target.
    Uses Gazebo convention where camera looks along a direction.
    
    Args:
        eye_x, eye_y, eye_z: Camera position
        target_x, target_y, target_z: Point to look at
    
    Returns:
        dict with quaternion components {x, y, z, w}
    """
    # Vector from eye to target (forward direction)
    dx = target_x - eye_x
    dy = target_y - eye_y
    dz = target_z - eye_z
    
    # Distance
    dist = math.sqrt(dx**2 + dy**2 + dz**2)
    
    if dist == 0:
        return {'x': 0, 'y': 0, 'z': 0, 'w': 1}
    
    # Normalize forward vector
    fx = dx / dist
    fy = dy / dist
    fz = dz / dist
    
    # Calculate pitch (rotation around Y-axis, looking up/down)
    # pitch = -asin(fz) because looking down means positive pitch
    pitch = -math.asin(np.clip(fz, -1.0, 1.0))
    
    # Calculate yaw (rotation around Z-axis, left/right)
    yaw = math.atan2(fy, fx)
    
    # Roll is 0 (no banking)
    roll = 0.0
    
    # Convert to quaternion
    return euler_to_quaternion(roll, pitch, yaw)

def take_screenshot(cmd='ign'):
    """Trigger internal screenshot service - saves to default location (~/.ignition/gui/pictures/)"""
    # Service: /gui/screenshot
    # Empty string -> saves to default location
    
    msg_pkg = "ignition.msgs" if "ign" in cmd else "gz.msgs"
    
    cmd_args = [
        cmd, 'service', '-s', '/gui/screenshot',
        '--reqtype', f'{msg_pkg}.StringMsg',
        '--reptype', f'{msg_pkg}.Boolean',
        '--timeout', '2000',
        '--req', 'data: ""'
    ]
    
    result = subprocess.run(cmd_args, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"Screenshot service failed: {result.stderr}")
        return False
    elif "data: false" in result.stdout.lower():
        print(f"Screenshot service returned false.")
        return False
    return True

def find_latest_file(directories, min_mtime):
    """Find the most recently modified file in a list of directories"""
    latest_file = None
    latest_time = 0
    
    for d in directories:
        if not os.path.exists(d):
            continue
            
        for root, dirs, files in os.walk(d):
            for f in files:
                if not f.endswith('.png'):
                    continue
                    
                fp = os.path.join(root, f)
                try:
                    mtime = os.path.getmtime(fp)
                    if mtime >= min_mtime and mtime > latest_time:
                        latest_time = mtime
                        latest_file = fp
                except Exception:
                    pass
                    
    return latest_file

def get_default_screenshot_dir(cmd='ign'):
    """Get the default directory where Gazebo saves screenshots"""
    home = os.path.expanduser("~")
    # The actual location is ~/.ignition/gui/pictures/ (or ~/.gz/gui/pictures/)
    return [
        os.path.join(home, ".ignition", "gui", "pictures"),
        os.path.join(home, ".gz", "gui", "pictures")
    ]


def get_default_video_dir():
    """Get the default directory where Gazebo saves videos"""
    home = os.path.expanduser("~")
    return [
        os.path.join(home, ".ignition", "gui", "videos"),
        os.path.join(home, ".gz", "gui", "videos"),
        home  # Sometimes saves to home directory
    ]


def add_video_recorder_to_sdf(sdf_file, output_sdf):
    """Add VideoRecorder GUI plugin and required scene plugins to an SDF file"""
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    
    # Find or create the world element
    world = root.find('world')
    if world is None:
        print("Warning: No <world> element found in SDF")
        return False
    
    # Check if gui element exists, remove it to rebuild properly
    gui = world.find('gui')
    if gui is not None:
        world.remove(gui)
    
    # Create new gui element with all required plugins
    gui = ET.SubElement(world, 'gui')
    gui.set('fullscreen', '0')
    
    # Helper to add property elements
    def add_props(parent, props):
        for key, typ, val in props:
            prop = ET.SubElement(parent, 'property')
            prop.set('key', key)
            prop.set('type', typ)
            prop.text = val
    
    # 1. MinimalScene plugin (required for rendering)
    minimal_scene = ET.SubElement(gui, 'plugin')
    minimal_scene.set('filename', 'MinimalScene')
    minimal_scene.set('name', '3D View')
    
    ms_ign_gui = ET.SubElement(minimal_scene, 'ignition-gui')
    add_props(ms_ign_gui, [('showTitleBar', 'bool', 'false'), ('state', 'string', 'docked')])
    
    engine = ET.SubElement(minimal_scene, 'engine')
    engine.text = 'ogre2'
    scene = ET.SubElement(minimal_scene, 'scene')
    scene.text = 'scene'
    ambient = ET.SubElement(minimal_scene, 'ambient_light')
    ambient.text = '0.4 0.4 0.4'
    bg_color = ET.SubElement(minimal_scene, 'background_color')
    bg_color.text = '0.8 0.8 0.8'
    camera_pose = ET.SubElement(minimal_scene, 'camera_pose')
    camera_pose.text = '-6 0 6 0 0.5 0'
    
    # 2. GzSceneManager plugin (required)
    scene_mgr = ET.SubElement(gui, 'plugin')
    scene_mgr.set('filename', 'GzSceneManager')
    scene_mgr.set('name', 'Scene Manager')
    sm_ign_gui = ET.SubElement(scene_mgr, 'ignition-gui')
    add_props(sm_ign_gui, [('state', 'string', 'floating'), ('showTitleBar', 'bool', 'false'),
                          ('resizable', 'bool', 'false'), ('width', 'double', '5'), ('height', 'double', '5')])
    
    # 3. InteractiveViewControl plugin (for camera control)
    view_ctrl = ET.SubElement(gui, 'plugin')
    view_ctrl.set('filename', 'InteractiveViewControl')
    view_ctrl.set('name', 'Interactive view control')
    vc_ign_gui = ET.SubElement(view_ctrl, 'ignition-gui')
    add_props(vc_ign_gui, [('state', 'string', 'floating'), ('showTitleBar', 'bool', 'false'),
                          ('resizable', 'bool', 'false'), ('width', 'double', '5'), ('height', 'double', '5')])
    
    # 4. CameraTracking plugin (provides /gui/move_to and /gui/move_to/pose)
    cam_tracking = ET.SubElement(gui, 'plugin')
    cam_tracking.set('filename', 'CameraTracking')
    cam_tracking.set('name', 'Camera Tracking')
    ct_ign_gui = ET.SubElement(cam_tracking, 'ignition-gui')
    add_props(ct_ign_gui, [('state', 'string', 'floating'), ('showTitleBar', 'bool', 'false'),
                          ('resizable', 'bool', 'false'), ('width', 'double', '5'), ('height', 'double', '5')])
    
    # 5. Screenshot plugin (provides /gui/screenshot)
    screenshot = ET.SubElement(gui, 'plugin')
    screenshot.set('filename', 'Screenshot')
    screenshot.set('name', 'Screenshot')
    ss_ign_gui = ET.SubElement(screenshot, 'ignition-gui')
    add_props(ss_ign_gui, [('state', 'string', 'floating'), ('showTitleBar', 'bool', 'false'),
                          ('resizable', 'bool', 'false'), ('width', 'double', '5'), ('height', 'double', '5')])
    
    # 6. VideoRecorder plugin
    video_recorder = ET.SubElement(gui, 'plugin')
    video_recorder.set('filename', 'VideoRecorder')
    video_recorder.set('name', 'VideoRecorder')
    
    vr_ign_gui = ET.SubElement(video_recorder, 'ignition-gui')
    add_props(vr_ign_gui, [('resizable', 'bool', 'false'), ('x', 'double', '300'), ('y', 'double', '50'),
                          ('width', 'double', '50'), ('height', 'double', '50'),
                          ('state', 'string', 'floating'), ('showTitleBar', 'bool', 'false'),
                          ('cardBackground', 'string', '#777777')])
    
    # VideoRecorder settings
    record_video = ET.SubElement(video_recorder, 'record_video')
    use_sim_time = ET.SubElement(record_video, 'use_sim_time')
    use_sim_time.text = 'false'
    lockstep = ET.SubElement(record_video, 'lockstep')
    lockstep.text = 'false'
    bitrate = ET.SubElement(record_video, 'bitrate')
    bitrate.text = '8000000'
    
    legacy = ET.SubElement(video_recorder, 'legacy')
    legacy.text = 'false'
    
    # Write modified SDF
    tree.write(output_sdf, encoding='unicode', xml_declaration=True)
    return True

def focus_on_model(model_name, cmd='ign'):
    """Focus camera on a named model using /gui/move_to service"""
    cmd_args = [
        cmd, 'service', '-s', '/gui/move_to',
        '--reqtype', 'ignition.msgs.StringMsg',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '3000',
        '--req', f'data: "{model_name}"'
    ]
    
    result = subprocess.run(cmd_args, capture_output=True, text=True)
    return result.returncode == 0

def read_camera_pose(cmd='ign', timeout=5.0):
    """Read current camera pose from /gui/camera/pose topic"""
    import re
    
    cmd_args = [cmd, 'topic', '-e', '-t', '/gui/camera/pose', '-n', '1']
    
    try:
        result = subprocess.run(cmd_args, capture_output=True, text=True, timeout=timeout)
        output = result.stdout
        
        # Extract position block
        pos_block = re.search(r'position\s*\{([^}]*)\}', output, re.DOTALL)
        orient_block = re.search(r'orientation\s*\{([^}]*)\}', output, re.DOTALL)
        
        if pos_block and orient_block:
            pos_text = pos_block.group(1)
            orient_text = orient_block.group(1)
            
            # Parse individual values (default to 0 if not present)
            def get_val(text, key):
                match = re.search(rf'{key}:\s*([-\d.e+]+)', text)
                return float(match.group(1)) if match else 0.0
            
            x = get_val(pos_text, 'x')
            y = get_val(pos_text, 'y')
            z = get_val(pos_text, 'z')
            
            qx = get_val(orient_text, 'x')
            qy = get_val(orient_text, 'y')
            qz = get_val(orient_text, 'z')
            qw = get_val(orient_text, 'w')
            
            return (x, y, z, qx, qy, qz, qw)
        else:
            print(f"  Could not find position/orientation blocks in output")
    except subprocess.TimeoutExpired:
        print(f"  Timeout reading camera pose")
    except Exception as e:
        print(f"  Failed to read camera pose: {e}")
    
    return None

def move_camera(x, y, z, roll, pitch, yaw, cmd='ign'):
    """Send service call to move camera to absolute pose (Euler angles)"""
    q = euler_to_quaternion(roll, pitch, yaw)
    return move_camera_quat(x, y, z, q['x'], q['y'], q['z'], q['w'], cmd)

def move_camera_quat(x, y, z, qx, qy, qz, qw, cmd='ign'):
    """Send service call to move camera using quaternion orientation directly"""
    # Construct Protobuf-like string for the service call
    req_str = (
        f'pose: {{ '
        f'position: {{ x: {x}, y: {y}, z: {z} }}, '
        f'orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }} '
        f'}}'
    )
    
    # Call the service (note: path is /gui/move_to/pose with slash, not underscore)
    cmd_args = [
        cmd, 'service', '-s', '/gui/move_to/pose',
        '--reqtype', 'ignition.msgs.GUICamera',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', req_str
    ]
    
    result = subprocess.run(cmd_args, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"  Camera move failed: {result.stderr[:100] if result.stderr else 'unknown error'}")

def wait_for_service(service_name, timeout=30, cmd='ign'):
    """Wait for a service to become available"""
    print(f"Waiting for service {service_name}...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            # List services
            output = subprocess.check_output([cmd, 'service', '-l'], stderr=subprocess.DEVNULL).decode()
            if service_name in output:
                print(f"Service {service_name} is available.")
                return True
        except Exception:
            pass
        time.sleep(1)
    print(f"Timeout waiting for {service_name}. Is the GUI loaded?")
    return False


# =============================================================================
# VIDEO RECORDING FUNCTIONS
# =============================================================================

def start_video_recording(output_file, video_format='mp4', cmd='ign'):
    """Start video recording using the GUI video recorder"""
    msg_pkg = "ignition.msgs" if "ign" in cmd else "gz.msgs"
    
    # The VideoRecorder uses a topic to control recording
    # We need to publish to start recording
    cmd_args = [
        cmd, 'topic', '-t', '/gui/record_video',
        '-m', f'{msg_pkg}.VideoRecord',
        '-p', f'start: true, format: "{video_format}", save_filename: "{output_file}"'
    ]
    
    result = subprocess.run(cmd_args, capture_output=True, text=True)
    return result.returncode == 0


def stop_video_recording(cmd='ign'):
    """Stop video recording"""
    msg_pkg = "ignition.msgs" if "ign" in cmd else "gz.msgs"
    
    cmd_args = [
        cmd, 'topic', '-t', '/gui/record_video',
        '-m', f'{msg_pkg}.VideoRecord',
        '-p', 'stop: true'
    ]
    
    result = subprocess.run(cmd_args, capture_output=True, text=True)
    return result.returncode == 0


def find_recorded_video(search_dirs, video_format='mp4', min_mtime=0):
    """Find the most recently recorded video file"""
    latest_file = None
    latest_time = 0
    
    for d in search_dirs:
        if not os.path.exists(d):
            continue
        
        try:
            for f in os.listdir(d):
                if f.endswith(f'.{video_format}'):
                    fp = os.path.join(d, f)
                    mtime = os.path.getmtime(fp)
                    if mtime >= min_mtime and mtime > latest_time:
                        latest_time = mtime
                        latest_file = fp
        except Exception:
            pass
    
    return latest_file


# =============================================================================
# MODULAR ANIMATION FUNCTIONS
# =============================================================================

def capture_single_frame(frame_index, frames_dir, cmd='ign'):
    """Capture a single screenshot and save it to frames_dir.
    
    Returns:
        str: Path to saved frame, or None if failed
    """
    frame_path = os.path.join(frames_dir, f"frame_{frame_index:03d}.png")
    
    # Get file count in default directories before screenshot
    search_dirs = get_default_screenshot_dir(cmd)
    files_before = {}
    for d in search_dirs:
        if os.path.exists(d):
            try:
                files_before[d] = set(os.listdir(d))
            except:
                files_before[d] = set()
    
    # Take screenshot
    success = take_screenshot(cmd)
    
    if not success:
        return None
    
    # Wait and look for new file in default directories
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


def animate_unzoom(start_x, start_y, start_z, qx, qy, qz, qw,
                   focal_x, focal_y, focal_z,
                   start_distance, end_distance,
                   num_frames, frames_dir, cmd='ign', settle_time=0.3):
    """
    Perform unzoom animation: move camera backwards along its view direction.
    
    Args:
        start_x/y/z: Initial camera position
        qx/qy/qz/qw: Camera orientation quaternion (fixed during unzoom)
        focal_x/y/z: Target point camera is looking at
        start_distance: Starting distance from focal point
        end_distance: Ending distance from focal point
        num_frames: Number of frames to capture
        frames_dir: Directory to save frames
        cmd: Gazebo command ('ign' or 'gz')
        settle_time: Time to wait after moving camera
    
    Returns:
        tuple: (list of captured frame paths, final camera position (x, y, z))
    """
    captured = []
    
    # Extract forward direction from quaternion
    # Camera looks along +X axis in body frame
    fwd_x = 1.0 - 2.0*(qy*qy + qz*qz)
    fwd_y = 2.0*(qx*qy + qz*qw)
    fwd_z = 2.0*(qx*qz - qy*qw)
    
    # Unzoom direction is opposite to forward
    dir_x, dir_y, dir_z = -fwd_x, -fwd_y, -fwd_z
    
    # Normalize
    dir_len = math.sqrt(dir_x**2 + dir_y**2 + dir_z**2)
    if dir_len > 0:
        dir_x /= dir_len
        dir_y /= dir_len
        dir_z /= dir_len
    
    print(f"  Unzoom: {num_frames} frames, distance {start_distance:.1f}m → {end_distance:.1f}m")
    
    cam_x, cam_y, cam_z = start_x, start_y, start_z
    
    for i in range(num_frames):
        # Linear interpolation of distance
        t = i / max(num_frames - 1, 1)
        new_dist = start_distance + (end_distance - start_distance) * t
        
        # Calculate camera position
        move_amount = new_dist - start_distance
        cam_x = start_x + (dir_x * move_amount)
        cam_y = start_y + (dir_y * move_amount)
        cam_z = start_z + (dir_z * move_amount)
        
        # Move camera with fixed orientation
        move_camera_quat(cam_x, cam_y, cam_z, qx, qy, qz, qw, cmd)
        time.sleep(settle_time)
        
        # Capture frame
        frame_idx = len(captured)
        frame_path = capture_single_frame(frame_idx, frames_dir, cmd)
        if frame_path:
            captured.append(frame_path)
            print(f"    Frame {i+1}/{num_frames}: dist={new_dist:.1f}m", end='\r')
    
    print()
    return captured, (cam_x, cam_y, cam_z)


def animate_orbit(focal_x, focal_y, focal_z,
                  orbit_radius, orbit_height,
                  start_angle, num_frames, frames_dir,
                  frame_offset=0, cmd='ign', settle_time=0.3):
    """
    Perform orbit animation: circle around focal point while looking at it.
    
    Args:
        focal_x/y/z: Center point to orbit around and look at
        orbit_radius: Radius of the circular orbit (in XY plane)
        orbit_height: Z height of camera during orbit
        start_angle: Starting angle in radians
        num_frames: Number of frames for one full rotation
        frames_dir: Directory to save frames
        frame_offset: Starting frame number (for continuing from previous animation)
        cmd: Gazebo command
        settle_time: Time to wait after moving camera
    
    Returns:
        list: Paths to captured frames
    """
    captured = []
    
    print(f"  Orbit: {num_frames} frames, radius={orbit_radius:.1f}m, height={orbit_height:.1f}m")
    
    for i in range(num_frames):
        # Calculate angle (one full circle = 2*pi)
        progress = i / num_frames
        theta = start_angle + (2 * math.pi * progress)
        
        # Calculate position on circle
        cam_x = focal_x + orbit_radius * math.cos(theta)
        cam_y = focal_y + orbit_radius * math.sin(theta)
        cam_z = orbit_height
        
        # Calculate orientation to look at focal point
        q = calculate_lookat_quaternion(cam_x, cam_y, cam_z, focal_x, focal_y, focal_z)
        
        # Move camera
        move_camera_quat(cam_x, cam_y, cam_z, q['x'], q['y'], q['z'], q['w'], cmd)
        time.sleep(settle_time)
        
        # Capture frame
        frame_idx = frame_offset + len(captured)
        frame_path = capture_single_frame(frame_idx, frames_dir, cmd)
        if frame_path:
            captured.append(frame_path)
            angle_deg = math.degrees(theta) % 360
            print(f"    Frame {i+1}/{num_frames}: angle={angle_deg:.0f}°", end='\r')
    
    print()
    return captured


def orbit_camera_for_video(focal_x, focal_y, focal_z,
                           orbit_radius, orbit_height,
                           start_angle, duration, cmd='ign', num_points=4):
    """
    Perform orbit animation for video recording using only key points.
    Camera moves through 4 points (0°, 90°, 180°, 270°) during full rotation.
    
    Args:
        focal_x/y/z: Center point to orbit around and look at
        orbit_radius: Radius of the circular orbit
        orbit_height: Z height of camera
        start_angle: Starting angle in radians
        duration: Total duration in seconds
        cmd: Gazebo command
        num_points: Number of key points for orbit (default 4)
    """
    step_time = duration / num_points
    
    print(f"  Orbit: {num_points} key points, duration={duration:.1f}s")
    
    for i in range(num_points + 1):
        progress = i / num_points
        theta = start_angle + (2 * math.pi * progress)
        
        cam_x = focal_x + orbit_radius * math.cos(theta)
        cam_y = focal_y + orbit_radius * math.sin(theta)
        cam_z = orbit_height
        
        q = calculate_lookat_quaternion(cam_x, cam_y, cam_z, focal_x, focal_y, focal_z)
        move_camera_quat(cam_x, cam_y, cam_z, q['x'], q['y'], q['z'], q['w'], cmd)
        
        angle_deg = math.degrees(theta) % 360
        print(f"    Point {i+1}/{num_points+1}: {angle_deg:.0f}°")
        
        if i < num_points:
            time.sleep(step_time)
    
    print()


def unzoom_camera_for_video(start_x, start_y, start_z, qx, qy, qz, qw,
                            start_distance, end_distance,
                            duration, cmd='ign', num_points=2):
    """
    Perform unzoom animation using only start and end points.
    
    Returns:
        tuple: Final camera position (x, y, z)
    """
    # Extract forward direction from quaternion
    fwd_x = 1.0 - 2.0*(qy*qy + qz*qz)
    fwd_y = 2.0*(qx*qy + qz*qw)
    fwd_z = 2.0*(qx*qz - qy*qw)
    
    # Unzoom direction is opposite to forward
    dir_x, dir_y, dir_z = -fwd_x, -fwd_y, -fwd_z
    
    # Normalize
    dir_len = math.sqrt(dir_x**2 + dir_y**2 + dir_z**2)
    if dir_len > 0:
        dir_x /= dir_len
        dir_y /= dir_len
        dir_z /= dir_len
    
    print(f"  Unzoom: {num_points} points, {start_distance:.1f}m → {end_distance:.1f}m")
    
    # Just move to start, then end
    step_time = duration / (num_points - 1) if num_points > 1 else duration
    
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        new_dist = start_distance + (end_distance - start_distance) * t
        
        move_amount = new_dist - start_distance
        cam_x = start_x + (dir_x * move_amount)
        cam_y = start_y + (dir_y * move_amount)
        cam_z = start_z + (dir_z * move_amount)
        
        move_camera_quat(cam_x, cam_y, cam_z, qx, qy, qz, qw, cmd)
        print(f"    Point {i+1}/{num_points}: dist={new_dist:.1f}m")
        
        if i < num_points - 1:
            time.sleep(step_time)
    
    print()
    return (cam_x, cam_y, cam_z)


# =============================================================================
# MAIN CAPTURE FUNCTION
# =============================================================================

def capture_video(sdf_file, output_video, duration=5.0, radius=15.0, height=10.0, 
                  video_format='mp4', cmd='ign'):
    """Launch Gazebo, perform orbit animation, record video"""
    
    print(f"\n{'='*60}")
    print(f"Processing {sdf_file}...")
    print(f"{'='*60}")
    
    # Kill any existing gazebo processes first
    print("Killing existing Gazebo processes...")
    kill_gazebo_processes()
    
    # Get world center and appropriate radius from SDF
    center_x, center_y, auto_radius = get_world_bounds_from_sdf(sdf_file)
    
    # Use auto-calculated radius if not specified
    if radius == 15.0:  # default value
        radius = auto_radius
    
    # Ensure we are using absolute paths for the SDF file
    sdf_abs_path = os.path.abspath(sdf_file)
    output_abs_path = os.path.abspath(output_video)
    
    # Create a temporary SDF with VideoRecorder plugin
    temp_sdf = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
    temp_sdf_path = temp_sdf.name
    temp_sdf.close()
    
    print(f"Adding VideoRecorder plugin to SDF...")
    if not add_video_recorder_to_sdf(sdf_abs_path, temp_sdf_path):
        print("Warning: Could not add VideoRecorder plugin, using original SDF")
        temp_sdf_path = sdf_abs_path
    
    # Launch Gazebo
    print(f"Launching Gazebo...")
    
    # Set NVIDIA environment variables for proper GPU rendering
    env = os.environ.copy()
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    
    gz_process = subprocess.Popen([cmd, 'gazebo', temp_sdf_path], 
                                   stdout=subprocess.DEVNULL, 
                                   stderr=subprocess.DEVNULL, 
                                   env=env)
    
    try:
        # Wait for Gazebo to load and services to be ready
        if not wait_for_service('/gui/move_to/pose', timeout=60, cmd=cmd):
            print("Skipping: GUI services not found.")
            return False

        # Wait for scene to fully render
        print("Waiting for scene to render...")
        time.sleep(4)
        
        # =====================================================================
        # PHASE 1: Focus on ground_plane to center camera
        # =====================================================================
        print("\nPHASE 1: Focusing on ground_plane (centering camera)...")
        focus_on_model("ground_plane", cmd)
        time.sleep(2.0)
        
        # Read camera pose after focus
        pose = read_camera_pose(cmd)
        
        if pose:
            start_x, start_y, start_z, qx, qy, qz, qw = pose
            print(f"  Camera position: ({start_x:.2f}, {start_y:.2f}, {start_z:.2f})")
        else:
            print("  Warning: Could not read camera pose, using defaults")
            start_x = center_x + radius * 0.5
            start_y = center_y + radius * 0.5
            start_z = radius * 0.5
            qx, qy, qz, qw = 0.0, 0.38, 0.0, 0.92
        
        # Focal point (center of map at ground level)
        focal_x, focal_y, focal_z = center_x, center_y, 0.0
        
        # Calculate starting distance
        dx = start_x - focal_x
        dy = start_y - focal_y
        dz = start_z - focal_z
        start_distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Split duration: 1/3 for unzoom, 2/3 for orbit
        unzoom_duration = duration / 3.0
        orbit_duration = duration - unzoom_duration
        
        # =====================================================================
        # PHASE 2: Start video recording
        # =====================================================================
        print(f"\nPHASE 2: Starting video recording...")
        
        # Record time before starting to find the new video file later
        record_start_time = time.time()
        
        # Start recording
        start_video_recording(output_abs_path, video_format, cmd)
        time.sleep(0.5)  # Give recorder time to start
        
        # =====================================================================
        # PHASE 3: Unzoom animation
        # =====================================================================
        print(f"\nPHASE 3: Unzoom animation ({unzoom_duration:.1f}s)...")
        
        end_distance = start_distance * 3.0  # Zoom out to 3x
        
        final_x, final_y, final_z = unzoom_camera_for_video(
            start_x, start_y, start_z,
            qx, qy, qz, qw,
            start_distance, end_distance,
            unzoom_duration, cmd
        )
        
        # =====================================================================
        # PHASE 4: Orbit animation
        # =====================================================================
        print(f"\nPHASE 4: Orbit animation ({orbit_duration:.1f}s)...")
        
        # Calculate orbit parameters from final unzoom position
        orbit_dx = final_x - focal_x
        orbit_dy = final_y - focal_y
        orbit_radius = math.sqrt(orbit_dx**2 + orbit_dy**2)
        orbit_height = final_z
        start_angle = math.atan2(orbit_dy, orbit_dx)
        
        # Ensure minimum orbit radius
        if orbit_radius < 5.0:
            orbit_radius = radius * 2.0
        
        print(f"  Orbit radius: {orbit_radius:.1f}m, height: {orbit_height:.1f}m")
        
        orbit_camera_for_video(
            focal_x, focal_y, focal_z,
            orbit_radius, orbit_height,
            start_angle, orbit_duration, cmd
        )
        
        # =====================================================================
        # PHASE 5: Stop recording and save
        # =====================================================================
        print(f"\nPHASE 5: Stopping video recording...")
        
        stop_video_recording(cmd)
        time.sleep(1.0)  # Give recorder time to finalize
        
        # Look for the recorded video file
        search_dirs = get_default_video_dir()
        search_dirs.append(os.path.dirname(output_abs_path))
        
        # Wait a bit and check if video was created
        time.sleep(2.0)
        
        # Check if output file exists
        if os.path.exists(output_abs_path):
            file_size = os.path.getsize(output_abs_path)
            print(f"  Video saved: {output_abs_path} ({file_size/1024/1024:.1f} MB)")
            return True
        else:
            # Try to find it in default directories
            found_video = find_recorded_video(search_dirs, video_format, record_start_time)
            if found_video:
                # Move to desired location
                shutil.move(found_video, output_abs_path)
                file_size = os.path.getsize(output_abs_path)
                print(f"  Video saved: {output_abs_path} ({file_size/1024/1024:.1f} MB)")
                return True
            else:
                print(f"  Warning: Video file not found. Check {search_dirs}")
                return False
        
    finally:
        # Kill Gazebo and all related processes
        print("\nClosing Gazebo...")
        gz_process.terminate()
        try:
            gz_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            gz_process.kill()
        
        # Force kill all related processes
        kill_gazebo_processes()
        
        # Clean up temp SDF
        if temp_sdf_path != sdf_abs_path and os.path.exists(temp_sdf_path):
            os.remove(temp_sdf_path)
        
        print("Done with this world.")

def main():
    parser = argparse.ArgumentParser(description="Generate videos from SDF files using Ignition Gazebo")
    parser.add_argument("input", nargs="?", default=None, help="Single SDF file or directory containing SDF files")
    parser.add_argument("-o", "--output", default=None, help="Output video file (for single SDF) or directory (for batch)")
    parser.add_argument("--duration", type=float, default=5.0, help="Video duration in seconds (default: 5.0)")
    parser.add_argument("--radius", type=float, default=15.0, help="Orbit radius")
    parser.add_argument("--height", type=float, default=10.0, help="Camera height")
    parser.add_argument("--format", default="mp4", choices=['mp4', 'ogv'], help="Video format (default: mp4)")
    parser.add_argument("--cmd", default="ign", help="Command prefix (ign or gz)")
    
    args = parser.parse_args()
    
    # Determine input path
    input_path = args.input if args.input else "maps/generated"
    
    # Check if input is a single file or directory
    if os.path.isfile(input_path):
        # Single file mode
        if not input_path.endswith('.sdf'):
            print(f"Error: {input_path} is not an SDF file")
            return
        
        # Determine output path
        if args.output:
            output_video = args.output
            # Ensure output directory exists
            output_dir = os.path.dirname(output_video)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir)
        else:
            base_name = os.path.splitext(os.path.basename(input_path))[0]
            output_video = f"videos/{base_name}.{args.format}"
            if not os.path.exists("videos"):
                os.makedirs("videos")
        
        print(f"Processing single file: {input_path} -> {output_video}")
        capture_video(input_path, output_video, args.duration, args.radius, args.height, 
                     args.format, args.cmd)
    
    elif os.path.isdir(input_path):
        # Directory/batch mode
        output_dir = args.output if args.output else "videos"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        sdf_files = glob.glob(os.path.join(input_path, "*.sdf"))
        sdf_files.sort()
        
        if not sdf_files:
            print(f"No SDF files found in {input_path}")
            return
            
        print(f"Found {len(sdf_files)} SDF files")
        
        for sdf_file in sdf_files:
            base_name = os.path.splitext(os.path.basename(sdf_file))[0]
            output_video = os.path.join(output_dir, f"{base_name}.{args.format}")
            
            if os.path.exists(output_video):
                print(f"Skipping {base_name} (video exists)")
                continue
                
            capture_video(sdf_file, output_video, args.duration, args.radius, args.height,
                         args.format, args.cmd)
    else:
        print(f"Error: {input_path} does not exist")

if __name__ == "__main__":
    main()
