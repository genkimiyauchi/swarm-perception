#!/bin/bash

# Script to run commands during the webviz user study.
#
# Usage: ./experiment.sh <mode>
#
#       where <mode> is {1, 2}

mode=$1

echo "Mode: $mode"

docker rm -f argos
echo "Starting docker container..."
cd ~/Documents/
docker run --name argos -it -p 3000:3000 -p 8000:8000 -p 5000:5000 -v $(pwd)/swarm-perception-results:/home/docker/swarm-perception/results -w /home/docker/swarm-perception genki15/swarm-perception python3 web_app/app.py -m $mode
