# TOWR Python Bindings - Build & Usage Guide

## Overview

The TOWR library now includes Python bindings via pybind11, allowing you to use the trajectory optimization framework directly from Python.

**Files Added:**
- `towr/python/towr_bindings.cpp` - pybind11 binding source
- `hopper_example.py` - Python example equivalent to C++ hopper_example.cc
- Updated `towr/CMakeLists.txt` - Build configuration for Python module

**Module Name:** `towr_cpp` (imported as `import towr_cpp`)

---

## Prerequisites

Install required dependencies in your Conda environment:

```bash
conda install -c conda-forge eigen ipopt pybind11 cmake
```

**On Windows with Visual Studio:**
```bash
# Ensure you have Visual Studio Build Tools or full Visual Studio installed
# The build system will automatically detect MSVC
```

---

## Build Instructions

### 1. Create a build directory
```bash
cd d:\1-Research\Co-optimization\codesign_optimization_legged_robots\towr\towr-python\towr
mkdir build
cd build
```

### 2. Configure with CMake

**Windows (using MSVC):**
```bash
cmake .. -G "Visual Studio 16 2019" -DCMAKE_BUILD_TYPE=Release
```

Or for newer Visual Studio versions:
```bash
cmake .. -G "Visual Studio 17 2022" -DCMAKE_BUILD_TYPE=Release
```

**Alternative - Ninja (faster):**
```bash
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
```

### 3. Build the Python module

```bash
# Using Visual Studio:
cmake --build . --config Release

# Or using Ninja:
ninja

# Or direct cmake build:
cmake --build . --config Release --target towr_cpp
```

### 4. Locate the compiled module

The Python module will be in one of these locations:
```
build/Release/towr_cpp.pyd              (Windows Release)
build/Debug/towr_cpp.pyd                (Windows Debug)
build/towr_cpp.pyd                      (Windows - may vary by generator)
build/towr_cpp.so                       (Linux/macOS)
```

---

## Testing the Installation

### Method 1: Run the Python Example
```bash
cd d:\1-Research\Co-optimization\codesign_optimization_legged_robots\towr\towr-python
python hopper_example.py
```

### Method 2: Test in Python Directly

```python
import sys
sys.path.insert(0, r'd:\1-Research\Co-optimization\codesign_optimization_legged_robots\towr\towr-python\towr\build\Release')

import towr_cpp as towr
import numpy as np

# Create a robot and setup
nlp = towr.NlpFormulation()
nlp.terrain = towr.FlatGround(0.0)
nlp.model = towr.RobotModel(towr.RobotType.Quadruped)
nlp.initial_base.p = np.array([0.0, 0.0, 0.5])
nlp.final_base.p = np.array([1.0, 0.0, 0.5])

print("✓ TOWR Python bindings working!")
print(f"Robot mass: {nlp.model.mass()}")
print(f"Max leg length: {nlp.model.max_leg_length()}")
```

---

## API Reference

### Classes Available

#### `RobotType` (Enum)
```python
towr.RobotType.Monoped     # 1-legged hopper
towr.RobotType.Biped       # 2-legged walker/runner
towr.RobotType.Quadruped   # 4-legged walker/trot
```

#### `RobotModel`
```python
robot = towr.RobotModel(towr.RobotType.Quadruped)
robot.max_leg_length()      # Returns float (meters)
robot.mass()                # Returns float (kg)
robot.num_ee()              # Returns int (number of end effectors)
```

#### `FlatGround`
```python
terrain = towr.FlatGround(height=0.0)
terrain.GetHeight(x, y)     # Returns float (height at x, y)
terrain.GetNormal(x, y)     # Returns normal vector
```

#### `State`
```python
state = towr.State()
state.p = np.array([x, y, z])      # Position
state.v = np.array([vx, vy, vz])  # Velocity
state.a = np.array([ax, ay, az])  # Acceleration
```

#### `Parameters`
```python
params = towr.Parameters()
params.ee_phase_durations = [[0.4, 0.2, 0.4, 0.2]]
params.ee_in_contact_at_start = [True]
```

#### `NlpFormulation` (Main solver setup)
```python
nlp = towr.NlpFormulation()
nlp.model = towr.RobotModel(towr.RobotType.Quadruped)
nlp.terrain = towr.FlatGround(0.0)
nlp.initial_base = towr.State()   # Starting pose/velocity
nlp.final_base = towr.State()     # Goal pose/velocity
nlp.params = towr.Parameters()    # Motion phases

# For actual optimization:
# nlp.GetVariableSets()   # Get optimization variables
# nlp.GetConstraints()    # Get motion constraints
# nlp.GetCosts()          # Get cost functions
```

---

## Example: Quadruped Trot

```python
import towr_cpp as towr
import numpy as np

# Setup problem
nlp = towr.NlpFormulation()
nlp.model = towr.RobotModel(towr.RobotType.Quadruped)
nlp.terrain = towr.FlatGround(0.0)

# Initial state (standing pose)
nlp.initial_base.p = np.array([0.0, 0.0, 0.5])
nlp.initial_base.v = np.array([0.0, 0.0, 0.0])

# Goal state (1m forward, same height)
nlp.final_base.p = np.array([1.0, 0.0, 0.5])
nlp.final_base.v = np.array([0.0, 0.0, 0.0])

# Trot gait: (stance, swing) for each leg
# Left front, Right hind  have same phase
# Right front, Left hind have same phase (offset)
nlp.params.ee_phase_durations = [
    [0.3, 0.2, 0.3, 0.2],  # LF: stance 0.3s, swing 0.2s
    [0.3, 0.2, 0.3, 0.2],  # RF: offset (opposite LF)
    [0.3, 0.2, 0.3, 0.2],  # LH: same as LF
    [0.3, 0.2, 0.3, 0.2],  # RH: same as RF
]
nlp.params.ee_in_contact_at_start = [True, False, False, True]

# Ready for optimization!
print("✓ Quadruped trot problem formulated")
print(f"  Robot mass: {nlp.model.mass()} kg")
print(f"  Total motion time: {sum(nlp.params.ee_phase_durations[0])} s")
```

---

## Troubleshooting

### "Module not found" or "ImportError"

1. **Check build directory path:**
   ```python
   import sys
   sys.path.insert(0, r'towr\build\Release')  # Adjust path as needed
   import towr_cpp
   ```

2. **Verify pybind11 is installed:**
   ```bash
   conda list | grep pybind11
   # Should show pybind11 2.6+ 
   ```

3. **Check if build succeeded:**
   ```bash
   cd build
   cmake --build . --config Release --verbose
   ```

### CMake can't find pybind11

```bash
# Manually specify pybind11 path
cmake .. -DPYBIND11_DIR=<path-to-pybind11-cmake>
```

### Compilation errors on Windows

- Ensure C++ build tools are installed: `Visual Studio Build Tools 2019` or later
- Delete `build/` directory and rebuild:
  ```bash
  rmdir /s build
  mkdir build
  cd build
  cmake ..
  ```

---

## Next Steps

To extend the Python bindings further:

1. **Expose the Ipopt solver** (pybind11/numpy compatible):
   - Implement SolverFactory bindings
   - Add trajectory result extraction

2. **Add spline interpolation** for trajectory output:
   - Bind Spline class for smooth trajectory evaluation
   - Export waypoints at arbitrary times

3. **Create visualization helpers**:
   - Export constraint/cost information in tabular form
   - Generate JSON-compatible trajectory format for web visualization

See `towr/python/towr_bindings.cpp` for the current binding implementation.
