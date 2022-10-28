source /opt/ros2/foxy/setup.bash
source ./SUV-Tools/ros2/install/setup.bash
echo suv | sudo -bS leapd
cd /home/suv/olympiad/SUV-Tools/ros2/src/leapmotion/leapmotion
python3 LeapControl.py
