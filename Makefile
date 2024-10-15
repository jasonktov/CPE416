ARCH := $(shell uname -m)

ifeq ($(ARCH), x86_64)
    CONTAINER_NAME := ambulantelab/cpe416:lab4-x86
else ifeq ($(ARCH), arm64)
    CONTAINER_NAME := ambulantelab/cpe416:lab4-arm
else
    CONTAINER_NAME = ambulantelab/cpe416:lab4-x86
endif

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
