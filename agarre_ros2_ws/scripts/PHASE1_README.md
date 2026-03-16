# Phase 1 - Preflight & ROS2 Installation Scripts

This directory contains automated scripts for **Phase 1** of the `agarre_ros2_ws` reconstruction plan.

## Overview

Phase 1 sets up the foundational software stack needed for the rest of the reconstruction:
- System readiness checks
- ROS 2 Jazzy installation
- Build tool verification
- Validation of installed components

## Quick Start

### Option 1: Automated Run (Recommended)

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws/scripts
./01_run_all_preflight.sh
```

This script orchestrates all Phase 1 steps with interactive prompts.

### Option 2: Step-by-Step Manual Execution

#### Step 1: Run Preflight Check
```bash
./01_preflight_check.sh
```
**Purpose**: Verify that the system meets minimum requirements before installing software.

**Output**: 
- Console report with pass/fail status
- Log file: `preflight_report_${TIMESTAMP}.txt`

**Expected Result**: 
- Status: `✓ PASS` or `✓ PROCEED WITH CAUTION` (0 failures)
- If PASS, proceed to Step 2

---

#### Step 2: Install ROS2 Jazzy
```bash
./01_install_ros2_jazzy.sh
```
**Purpose**: Install ROS 2 Jazzy desktop distribution and build tools.

**Duration**: 10-15 minutes (depends on internet speed)

**Output**:
- Console progress with colored indicators
- Log file: `install_ros2_jazzy_${TIMESTAMP}.log`

**What Gets Installed**:
- `ros-jazzy-desktop` (full ROS2 Jazzy distribution)
- `python3-colcon-common-extensions` (build system)
- `python3-rosdep` (dependency manager)
- Build tools: gcc, g++, cmake, make

**Post-Install Note**: 
The ROS2 environment needs to be sourced. The script outputs instructions; you can add this to `~/.bashrc`:
```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

#### Step 3: Validate Installation
```bash
./01_validate_installation.sh
```
**Purpose**: Verify that all Phase 1 components installed correctly.

**Output**:
- Console validation report with detailed pass/fail status
- Log file: `validate_phase1_${TIMESTAMP}.txt`

**Expected Result**:
- Status: `✓ VALIDATION PASSED` (0 failures)
- All core ROS2 checks passing
- colcon build tool available
- rosdep dependency manager working

---

## Script Details

### 01_preflight_check.sh
Performs 10 core checks:
1. **OS Version**: Ubuntu 24.04 (warns if different)
2. **Python Version**: >= 3.10
3. **Build Tools**: gcc, g++, make, cmake
4. **Git**: Version control tool
5. **ROS2 Jazzy**: `/opt/ros/jazzy/setup.bash` exists
6. **colcon**: Build system available
7. **Gazebo**: Simulation engine (optional)
8. **Workspace Structure**: `src/` and `scripts/` directories exist
9. **Disk Space**: >= 20GB available
10. **venv**: Python virtual environment support

**Output Format**: Color-coded results with detailed report

---

### 01_install_ros2_jazzy.sh
Performs 8 installation steps:
1. Update package manager
2. Install build tools (gcc, cmake, git, etc.)
3. Setup ROS2 GPG keyring
4. Add ROS2 APT repository
5. Update from ROS2 repository
6. Install ROS2 Jazzy desktop
7. Install ROS2 development tools
8. Initialize rosdep

**Note**: Requires `sudo` access for system-level package installation

---

### 01_validate_installation.sh
Performs 12 detailed validation checks:

**Core ROS2 Checks**:
- setup.bash exists
- ROS_DISTRO environment variable
- ros2 CLI available
- ROS2 core packages installed

**Build Tools**:
- colcon build system
- colcon CMake extensions

**Dependency Management**:
- rosdep tool
- rosdep sources initialized

**System Packages**:
- Python3 packages (pip, venv)
- Build chain (gcc, g++, make, cmake)

**Optional Components**:
- Gazebo simulator (warning if not installed)
- MoveIt motion planning (warning if not installed)

---

### 01_run_all_preflight.sh
Meta-script that orchestrates all three scripts in sequence with:
- Progress indicators
- User confirmation prompts
- Error handling and exit codes
- Summary of next steps

---

## Environment Setup After Phase 1

After successful completion, set up your environment:

### For Current Session
```bash
source /opt/ros/jazzy/setup.bash
```

### For Permanent Setup (Recommended)
```bash
cat >> ~/.bashrc << 'EOF'

# ROS2 Jazzy Setup
source /opt/ros/jazzy/setup.bash
EOF

# Apply changes
source ~/.bashrc
```

### Verify Setup
```bash
echo $ROS_DISTRO  # Should output: jazzy
which ros2        # Should output: /opt/ros/jazzy/bin/ros2
```

---

## Troubleshooting

### Issue: "Preflight checks failed"
**Solution**: Review the preflight report and install missing components:
```bash
# Check which components are missing
cat preflight_report_*.txt

# Install build-essential if missing
sudo apt-get install build-essential

# Reinstall Python venv if missing
sudo apt-get install python3-venv
```

### Issue: "Permission denied" when running scripts
**Solution**: Make scripts executable:
```bash
chmod +x 01_*.sh
```

### Issue: ROS2 installation takes too long
**Solution**: This is normal for the first installation (10-15 minutes). The apt-get repository download is the bottleneck. If interrupted, simply re-run the install script; it will continue from where it left off.

### Issue: "rosdep init failed" or permission errors
**Solution**: These are typically non-critical. The script handles this gracefully. If rosdep is needed for Phase 2, you can manually run:
```bash
sudo rosdep init
rosdep update
```

### Issue: Validation shows warnings for optional components
**Solution**: Warnings (⚠) are OK to proceed. Only FAILURES (✗) block advancement. Optional components like Gazebo and MoveIt can be installed later if needed.

---

## Next Steps After Phase 1 ✓ PASS

Once Phase 1 completes successfully:

1. **Proceed to Phase 2**: Workspace Structure & Description
   ```bash
   cd /home/laboratorio/TFM/agarre_ros2_ws
   cat docs/rebuild_plan.md | grep -A 30 "PHASE 2:"
   ```

2. **Set up environment permanently**:
   ```bash
   echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Verify environment**:
   ```bash
   ros2 --version
   colcon --version
   ```

---

## Log Files

All scripts generate timestamped log files in the workspace root:
- `preflight_report_${TIMESTAMP}.txt`
- `install_ros2_jazzy_${TIMESTAMP}.log`
- `validate_phase1_${TIMESTAMP}.txt`

These logs are useful for troubleshooting or documentation. Review them if any step fails.

---

## Estimated Total Time

| Script | Time | Notes |
|--------|------|-------|
| 01_preflight_check.sh | 2 min | Fast system checks |
| 01_install_ros2_jazzy.sh | 10-15 min | Depends on internet speed |
| 01_validate_installation.sh | 3 min | Verification only |
| **Total (01_run_all_preflight.sh)** | **15-20 min** | Including prompts |

---

## Support & Documentation

- **Rebuild Plan**: [../docs/rebuild_plan.md](../docs/rebuild_plan.md)
- **ROS2 Documentation**: https://docs.ros.org/en/jazzy/
- **Troubleshooting**: [../docs/reconstruction/TROUBLESHOOTING.md](../docs/reconstruction/TROUBLESHOOTING.md) (created in Phase 9)

