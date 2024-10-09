ARCH := $(shell uname -m)

ifeq ($(ARCH), x86_64)
    CONTAINER_NAME := osrf/ros:humble-desktop
else ifeq ($(ARCH), arm64)
    CONTAINER_NAME := arm64v8/ros:humble
else
    CONTAINER_NAME = unknown
endif

# If when you run the command 'make arch' you get as output 'unknown'
# then uncomment one of the following lines, depending on the your architecture 
# CONTAINER_NAME := osrf/ros:humble-desktop
# CONTAINER_NAME := arm64v8/ros:humble

build: 
	docker build . -t ${IMAGE_NAME}

novnc:
	docker run -d --rm --net=ros \
	--env="DISPLAY_WIDTH=3000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--name=novnc -p=8080:8080 \
	theasp/novnc:latest

bash:
	docker run -it --name ${NAME} \
	--net=ros \
	--env="DISPLAY=novnc:0.0" \
	-v ./ros_ws:/CPE416/ros_ws:Z \
	${CONTAINER_NAME} \

arch:
	@echo "Make will pull the following image: ${CONTAINER_NAME}"
