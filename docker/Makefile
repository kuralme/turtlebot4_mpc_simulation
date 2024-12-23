# Makefile for Turtlebot4 sim docker image
#
# Usage(Currently no args):
#   make <target> <arg-name>=<arg-value>
#
# Examples:
#   make
#   make build-base

IMAGE_NAME = tb4_mpc
DOCKERFILE1 = ${PWD}/Dockerfile_ipopt
DOCKERFILE2 = ${PWD}/Dockerfile_ros2

# linux/arm64 for board, amd64 for pc
DEVICE_ARCH := $(shell dpkg --print-architecture)
ifeq ($(DEVICE_ARCH),arm64)
  DEVICE_ARCH := linux/arm64
endif

#####################################################################
#						BUILD TARGETS								#
#####################################################################
# Build the base image
.PHONY: build-base
build-base:
	@docker build --platform ${DEVICE_ARCH} -f ${DOCKERFILE1} -t ${IMAGE_NAME}_base ..

# Build the overlay image & remove the base
.PHONY: build-overlay
build-overlay: build-base
	@docker build --platform ${DEVICE_ARCH} -f ${DOCKERFILE2} -t ${IMAGE_NAME} ..
	@docker rmi ${IMAGE_NAME}_base

# Kill any running Docker containers
.PHONY: kill
kill:
	@echo "Stopping all running Docker containers:"
	@docker ps -q --filter ancestor=${IMAGE_NAME} | xargs -r docker kill

# Docker compose pre-start script
.PHONY: up
up:
	@echo "Starting the Docker container..."
	@bash pre_start.sh

# Build all targets if not specified
.DEFAULT_GOAL := all
all: build-base build-overlay up
