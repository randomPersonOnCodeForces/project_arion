#!/bin/bash
# AI Academy Warehouse Demo - Pre-Demo Verification Script
# Run this script before Thursday to ensure everything is ready

set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}AI Academy Warehouse - Pre-Demo Check${NC}"
echo -e "${BLUE}=========================================${NC}\n"

# Check 1: Workspace structure
echo -e "${YELLOW}[1/8] Checking workspace structure...${NC}"
if [ -d "src/arion_simulation" ] && [ -d "src/aws-robomaker-small-warehouse-world" ]; then
    echo -e "${GREEN}✓ Workspace structure OK${NC}"
else
    echo -e "${RED}✗ Workspace structure NOT OK - run from project root${NC}"
    exit 1
fi

# Check 2: Required files exist
echo -e "${YELLOW}[2/8] Checking required files...${NC}"
REQUIRED_FILES=(
    "src/arion_simulation/launch/warehouse.launch.py"
    "src/arion_simulation/worlds/warehouse.sdf"
    "src/arion_simulation/urdf/turtlebot3_waffle_pi.urdf.xacro"
    "src/arion_simulation/config/warehouse.rviz"
    "package.xml"
    "README.md"
)

ALL_FILES_EXIST=true
for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}  ✓ $file${NC}"
    else
        echo -e "${RED}  ✗ $file (MISSING)${NC}"
        ALL_FILES_EXIST=false
    fi
done

if [ "$ALL_FILES_EXIST" = false ]; then
    echo -e "${RED}✗ Some required files are missing${NC}"
    exit 1
fi
echo -e "${GREEN}✓ All required files found${NC}"

# Check 3: Python syntax
echo -e "${YELLOW}[3/8] Checking Python syntax...${NC}"
if python3 -m py_compile src/arion_simulation/launch/warehouse.launch.py; then
    echo -e "${GREEN}✓ Launch file syntax OK${NC}"
else
    echo -e "${RED}✗ Launch file has syntax errors${NC}"
    exit 1
fi

# Check 4: ROS 2 environment
echo -e "${YELLOW}[4/8] Checking ROS 2 installation...${NC}"
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}✓ ROS 2 command found${NC}"
    ROS2_DISTRO=$(ros2 --version | grep -oP 'ROS 2 \K[^ ]+')
    echo -e "${GREEN}  → ROS 2 distribution: $ROS2_DISTRO${NC}"
else
    echo -e "${RED}✗ ROS 2 not found - run: source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

# Check 5: Gazebo installation
echo -e "${YELLOW}[5/8] Checking Gazebo installation...${NC}"
if command -v ign &> /dev/null; then
    echo -e "${GREEN}✓ Ignition Gazebo found${NC}"
    IGN_VERSION=$(ign gazebo --version)
    echo -e "${GREEN}  → Version: $IGN_VERSION${NC}"
else
    echo -e "${RED}✗ Ignition Gazebo not found${NC}"
    exit 1
fi

# Check 6: Build status (if build directory exists)
echo -e "${YELLOW}[6/8] Checking build status...${NC}"
if [ -d "build/arion_simulation" ]; then
    echo -e "${GREEN}✓ Build directory exists${NC}"
    if [ -d "install/arion_simulation" ]; then
        echo -e "${GREEN}✓ Install directory exists${NC}"
    else
        echo -e "${YELLOW}⚠ Install directory missing - run 'colcon build --symlink-install'${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Build directory missing - run 'colcon build --symlink-install'${NC}"
fi

# Check 7: Environment sourcing
echo -e "${YELLOW}[7/8] Checking if environment is sourced...${NC}"
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}✗ ROS 2 not in PATH - environment may not be sourced${NC}"
else
    SOURCE_CHECK=$(bash -c 'source install/setup.bash 2>&1 && echo "OK"')
    if [[ $SOURCE_CHECK == "OK" ]] || [ -f "install/setup.bash" ]; then
        echo -e "${GREEN}✓ Setup script exists and can be sourced${NC}"
    else
        echo -e "${YELLOW}⚠ May need to run: source install/setup.bash${NC}"
    fi
fi

# Check 8: Package availability
echo -e "${YELLOW}[8/8] Checking ROS 2 package availability...${NC}"
PACKAGES_OK=true

for pkg in "arion_simulation" "aws_robomaker_small_warehouse_world"; do
    if ros2 pkg prefix "$pkg" &> /dev/null; then
        PKG_PATH=$(ros2 pkg prefix "$pkg")
        echo -e "${GREEN}  ✓ $pkg found at: $PKG_PATH${NC}"
    else
        echo -e "${YELLOW}  ⚠ $pkg not found (may need: source install/setup.bash)${NC}"
        PACKAGES_OK=false
    fi
done

# Optional packages
for pkg in "teleop_twist_keyboard" "ros_gz_bridge" "rviz2"; do
    if ros2 pkg prefix "$pkg" &> /dev/null; then
        echo -e "${GREEN}  ✓ $pkg installed${NC}"
    else
        echo -e "${YELLOW}  ⚠ $pkg not installed (optional, but recommended)${NC}"
    fi
done

# Summary
echo -e "\n${BLUE}=========================================${NC}"
echo -e "${BLUE}Pre-Demo Verification Complete${NC}"
echo -e "${BLUE}=========================================${NC}\n"

if [ "$PACKAGES_OK" = false ]; then
    echo -e "${YELLOW}⚠ Action Required:${NC}"
    echo "  1. Run: source install/setup.bash"
    echo "  2. Run this script again"
else
    echo -e "${GREEN}✓ Everything looks good!${NC}"
    echo -e "\n${BLUE}To launch the demo, run:${NC}"
    echo "  source install/setup.bash"
    echo "  ros2 launch arion_simulation warehouse.launch.py"
fi

echo ""
