# 🎬 CineRoboSim

**Automated Cinematography for Robotics Simulators (Gazebo & Isaac Sim)**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Platform](https://img.shields.io/badge/Platform-Gazebo%20%7C%20Isaac%20Sim-orange)](https://github.com/)

**CineRoboSim** is a Python package designed to cure "shaky screen-recording syndrome." It automates camera movements, rendering settings, and frame capture in **Ignition/Gazebo** and **NVIDIA Isaac Sim** to produce publication-quality GIFs and videos of your robotic simulations.

---

## 📦 Features

### 🌌 Gazebo (Ignition / GZ Sim)
* **Smart Orbit:** Perfectly circular camera orbits around your robot or map center.
* **Linear Dolly (Unzoom):** Smooth vector-based camera pull-backs (dolly out) without gimbal lock.
* **Drift Correction:** Uses Quaternion math to ensure the camera stays locked on the target regardless of altitude.
* **Service Wrapper:** Handles the messy internal Gazebo `/gui/move_to/pose` and screenshot services automatically.
* **Flexible Playback:** Generate GIFs with reverse playback or smooth "ping-pong" loops.
* **Batch Processing:** Automatically process entire directories of SDF world files.

### 🪐 NVIDIA Isaac Sim (Coming Soon)
* **RTX Capture:** Automates switching to Path Tracing for photorealistic renders.
* **Timeline Sync:** Synchronizes robot USD animation with camera keyframes.
* **Bezier Paths:** Define complex fly-through paths using control points.

---

## 🚀 Installation

**Clone the repository:**
```bash
git clone https://github.com/Ali-J-Fesharaki/CineRoboSim.git
cd CineRoboSim
```

**Dependencies:**

The scripts only require two Python packages beyond the standard library:
```bash
pip install numpy pillow
```

**Prerequisites:**
- Python 3.8+
- Ignition Gazebo (Garden/Harmonic) or Gazebo Sim installed and configured
- A working GPU with proper display/rendering support

That's it! No complex setup required.

---

## 🎬 Usage

### Generate GIFs (`sdf_to_gif.py`)

Create animated GIFs from your SDF worlds.

**Basic Usage:**
```bash
python3 sdf_to_gif.py /path/to/world.sdf
```

**Batch Process a Directory:**
```bash
python3 sdf_to_gif.py maps/generated --output gifs
```

**Advanced Options:**
```bash
# Create a smooth looping GIF (forward then backward)
python3 sdf_to_gif.py world.sdf --pingpong

# Create a reversed GIF
python3 sdf_to_gif.py world.sdf --reverse

# Customize camera orbit
python3 sdf_to_gif.py world.sdf --radius 20.0 --height 15.0 --frames 90
```

### Generate Videos (`sdf_to_video.py`)

Record high-quality MP4 or OGV videos using Gazebo's internal recorder.

```bash
# Generate a 10-second MP4 video
python3 sdf_to_video.py world.sdf --duration 10.0 --format mp4

# Batch process
python3 sdf_to_video.py maps/generated --output videos
```

---

## 🗺️ Future Tools & Roadmap

Since we already have Orbit and Zoom for Gazebo, here is what we are building next to make this package indispensable:

### For Isaac Sim (NVIDIA Omniverse)
Isaac Sim is more powerful graphically than Gazebo. We plan to leverage that:
* **The "RTX Toggle":** A script to automatically switch the renderer from "Real-Time" to "Path Tracing" (Ray Tracing) just before capturing to get photorealistic shadows/reflections.
* **Replicator Integration:** Use `omni.replicator` to capture ground truth data (segmentation masks, depth) alongside the aesthetic video.
* **Sun Study:** A tool to animate the sun position (time of day) to show how the robot creates shadows over time.

### General Camera Moves (Math-based)
* **Bezier Fly-Through:** Instead of a circle or a straight line, let the user define 3 points. The camera smoothly flies through them using a Bezier curve (essential for drone footage).
* **The "Vertigo" Shot (Dolly Zoom):** Move the camera backward while zooming the FOV in. It creates a warping background effect while the robot stays the same size.
* **Third-Person Follow:** A function that takes a generic `target_frame` (like `base_link`) and applies a "spring-damper" smoothing to the camera so it follows the robot loosely (like a video game camera) rather than being rigidly attached.

---

## 🎥 Call for Contributors: Filmmakers & Artists

**We need your eye!**

Most robotics engineers know quaternions but don't know *composition*. We are actively seeking contributions from:
* **Cinematographers & Directors:** How should a camera move to make a robot look "fast" vs "stable"? What are the standard shot lists for technical demos?
* **3D Artists:** Help us create better lighting presets and material overrides for simulation environments.
* **Video Editors:** Advice on frame rates, codecs, and color grading pipelines for simulation footage.

If you have expertise in visual storytelling, please open an issue or discussion! We want to encode your artistic knowledge into our software tools.

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
