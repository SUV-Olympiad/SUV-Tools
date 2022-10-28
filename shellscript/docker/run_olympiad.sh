unset DOCKER_HOST

PS_NAME=olympiad
xhost +

docker stop $PS_NAME
docker rm $PS_NAME

docker run --gpus all -i --rm --privileged \
-e DISPLAY=$DISPLAY \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-v /home/$USER/olympiad:/home/suv/olympiad \
-v /dev:/dev:rw \
--group-add dialout \
--workdir /home/suv/olympiad/SUV-Tools/shellscript \
--hostname $(hostname) \
--network host \
--shm-size 8G \
--name $PS_NAME mywww5371/suv:1.6 ./run_suv.sh
