# update
sudo apt update
sudo apt upgrade -y

# environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# create workspace
cd ~
git clone https://github.com/OsawaKousei/ros2_serial_visualizer_ws.git
echo "source ~/ros2_serial_visualizer_ws/ros2_serial_visualizer/install/setup.bash" >> ~/.bashrc
cd ~/ros2_serial_visualizer_ws/ros2_serial_visualizer
colcon build
cd ~

# refresh bashrc
source ~/.bashrc
