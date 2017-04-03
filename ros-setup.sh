echo "Version:"
read version
echo ""

echo "Setting up ROS..."

source /opt/ros/$version/setup.sh
source ../devel/setup.sh
clear
echo "... ROS ready to go"
