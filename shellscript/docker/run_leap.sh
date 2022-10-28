unset DOCKER_HOST

PS_NAME=leap
xhost +

docker stop $PS_NAME
docker rm $PS_NAME

docker run --rm --gpus all -i --privileged \
-e DISPLAY=$DISPLAY \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-v /home/$USER/olympiad:/home/suv/olympiad \
-v /dev:/dev:rw \
-v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket \
-u suv \
--workdir /home/suv/olympiad/SUV-Tools/shellscript \
--group-add dialout \
--hostname $(hostname) \
--network host \
--shm-size 4096m \
--name $PS_NAME mywww5371/suv:1.6 ./run_leap.sh
