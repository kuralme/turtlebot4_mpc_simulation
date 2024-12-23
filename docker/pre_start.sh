#!/bin/bash

# Allow Docker containers to access the X server
xhost +local:docker

# Start Docker Compose
docker compose up