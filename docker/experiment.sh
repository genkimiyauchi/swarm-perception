#!/bin/bash

# Script to run commands during the webviz user study.

docker rm -f argos
echo "Starting docker container..."
cd ~/Documents/
docker run --name argos -it -p 3000:3000 -p 8000:8000 -p 5000:5000 -w /home/docker/swarm-perception genki15/swarm-perception python3 web_app/app.py
