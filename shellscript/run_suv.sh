#!/bin/bash
function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
	pkill MicroXRCEAgent
	pkill suvgcs
	pkill suv_run.sh
}
source /opt/ros/foxy/setup.bash

cd ../../PX4-Autopilot && echo shutdown | make px4_sitl_rtps gazebo
cd Tools/

cd ../../SUV-Tools/ros2 && colcon build --symlink-install
source install/setup.bash

cd ../../SUV-GCS
if [ ! -d "build" ]; then
 mkdir build
fi
cd build
cmake ..
make
./suvgcs &

MicroXRCEAgent udp4 -p 5000 &
../../PX4-Autopilot/Tools/suv_run.sh -t px4_sitl_rtps -f /home/suv/olympiad/PX4-Autopilot/Tools/vehicles -w suv

trap "cleanup" SIGINT SIGTERM EXIT
