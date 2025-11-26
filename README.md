# KINEMATIC-CHAINS │ Geometric Motion Abstraction

“Motion is geometry over time. Control is the mastery of that geometry.”

---

## 📐 Project Scope

KINEMATIC-CHAINS is the mathematical simulation engine and procedural rigging laboratory for the 073145 ecosystem.
While mecha-blocks handles the physical actuation (motors, voltage, mass), KINEMATIC-CHAINS handles the Abstract Planning of that motion.


It provides the solvers, rigs, and geometric transformations required to translate a desired end-effector position
$(x, y, z)$  

into specific joint angles
$(\theta_1, \theta_2, \dots)$


---

### Core Competencies

- Inverse Kinematics (IK): Analytical and iterative (Jacobian) solvers for multi-DOF robotic arms.

-  Procedural Rigging: Python scripts utilizing the bpy (Blender API) to generate skeletons and weight-painting for spatial-mechanics assets automatically.

- Path Planning: Bézier and spline interpolation for smooth trajectory generation.

---

## 🏗️ Modules & Architecture

The repository is divided into logical domains of motion:

1. solvers-py (The Math)

Function: Pure Python implementations of kinematic chains.
Algorithms:

- CCD (Cyclic Coordinate Descent): Fast, heuristic iterative solving.

- Analytical IK: Exact solutions for standard 6-DOF anthropomorphic arms (e.g., PUMA 560).

- DH-Parameters: Standard Denavit–Hartenberg matrices for describing chain topology.

2. rigging-blender (The Visuals)

Function: Automation for 3D assets.
Tools: Blender Python API.
Usage: Generates armatures for mecha-blocks prototypes, enabling visual validation in the argus-dashboard before physical assembly.

3. trajectory-planner (The Flow)

Function: Time-based interpolation.
Math: SLERP (Spherical Linear Interpolation) for quaternions to avoid gimbal lock during rotation.

---

## 🛠️ Tech Stack

- Simulation: Blender 4.0+ (bpy), PyBullet

- Math Core: NumPy, SciPy (for optimization constraints)

- Visualization: Three.js (WebGL exports), Matplotlib

---

## 📦 Usage Example

Procedural Armature Generation (Blender API)
~~~
import bpy
import math

def create_chain(length, segments):
    bpy.ops.object.armature_add(enter_editmode=True, location=(0, 0, 0))
    armature = bpy.context.object
    armature.name = "Procedural_Manipulator"
    
    bones = armature.data.edit_bones
    
    # Generate chain segments
    for i in range(segments):
        bone = bones.new(f"Link_{i}")
        bone.head = (0, 0, i * length)
        bone.tail = (0, 0, (i + 1) * length)
        if i > 0:
            bone.parent = bones[f"Link_{i-1}"]
            
    print(f"[*] Generated Kinematic Chain with {segments} segments.")

create_chain(length=0.5, segments=6)
~~~

Inverse Kinematics Solve (NumPy)

~~~
import numpy as np

def solve_forward_kinematics(joint_angles, link_lengths):
    # Simplified 2D Planar FK
    x = link_lengths[0] * np.cos(joint_angles[0]) + \
        link_lengths[1] * np.cos(joint_angles[0] + joint_angles[1])
    y = link_lengths[0] * np.sin(joint_angles[0]) + \
        link_lengths[1] * np.sin(joint_angles[0] + joint_angles[1])
    return np.array([x, y])
~~~

---

## 📡 Integration & Use Cases

Designed to function as a standalone mathematical backbone, this library integrates into agnostic robotic architectures:

- Cognitive Control (Upstream): Ingests target vectors and intent states from high-level AI agents or computer vision pipelines, converting abstract goals into geometric constraints.

- Firmware Abstraction (Downstream): Sanitizes and outputs joint-angle arrays to real-time executives or microcontrollers, ensuring that physical actuators receive mathematically valid instructions.

- Simulation & Visualization (Side-stream): Exports procedural armatures and GLTF models to UI layers, enabling "Digital Twin" capabilities for remote monitoring and safety validation.

---

## ⚖️ License

MIT License — Geometry is universal.
