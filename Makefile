CONTAINER_NAME := arm64v8/ros:humble


build: 
	docker build . -t ${CONTAINER_NAME}

bash:
	docker run -it --rm --name ${name} \
	-v ./ros_ws:/CPE416/ros_ws:Z \
	${CONTAINER_NAME} \

# Run this when using foxglove studio
# docker run -ti --rm --name $(name) -p 8765:8765
