CONTAINER_NAME := ambulantelab/lab1arm


build: 
	docker build . -t ${CONTAINER_NAME}

bash:
	docker run -ti --rm --name $(name) \
	-v ./ros_ws:/CPE416/ros_ws:Z \
	${CONTAINER_NAME} \

# Run this when using foxglove studio
# docker run -ti --rm --name $(name) -p 8765:8765