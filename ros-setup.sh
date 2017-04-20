echo "Setting up ROS Indigo..."
source /opt/ros/indigo/setup.sh || echo "No ROS Indigo distribution found."
source $(pwd)/devel/setup.sh
echo "... Done!"
