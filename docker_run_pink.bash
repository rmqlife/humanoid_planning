xhost +local:root

# Set up X11 socket for display forwarding
XSOCK=/tmp/.X11-unix

docker run -it --rm \
 -e DISPLAY=$DISPLAY \
 -v "$(pwd)":/workspace \
 -w /workspace \
 -v $XSOCK:$XSOCK \
 -e XAUTHORITY=/tmp/.Xauthority \
 -v $HOME/.Xauthority:/tmp/.Xauthority:rw \
 --privileged \
 --net=host \
 --ipc=host \
 --cap-add=SYS_NICE \
 --name fourier_aurora_sdk \
 fourier_aurora_sdk:pink bash
