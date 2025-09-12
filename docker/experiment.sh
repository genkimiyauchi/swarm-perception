#!/bin/bash

# Script to run commands during the webviz user study.
#
# Usage: ./experiment.sh <comamnd> <mode>
#
#       where <command> is {run, save}
#       where <mode> is {1, 2} (only when command is "run")

if [ $# -eq 0 ]; then
    echo "No command provided. Usage: ./experiment.sh <command={run,save}> <group={1,2}(only for run)>"
    exit 1
fi

command=$1
mode=$2

echo "Command: $command"

if [ "$command" = "run" ]; then
    echo "Mode: $mode"

    docker rm -f argos
    echo "Starting docker container..."
    cd ~/Documents/
    docker run --name argos -it -p 3000:3000 -p 8000:8000 -p 5000:5000 -v $(pwd)/swarm-perception-results:/home/docker/swarm-perception/results -w /home/docker/swarm-perception genki15/swarm-perception python3 web_app/app.py -m $mode

elif [ "$command" = "save" ]; then

    RESULTS_DIR=~/Documents/swarm-perception-results

    FILE_NAME=$(ls $RESULTS_DIR | grep .txt | head -n 1 | sed 's/\.txt//')

    # Check if the folder already exists
    if [ -d "$RESULTS_DIR/$FILE_NAME" ]; then
        echo "Directory $RESULTS_DIR/$FILE_NAME already exists. Do you want to overwrite its contents? (y/n)"
        read -r response
        if [[ "$response" != "y" ]]; then
            echo "Exiting without saving."
            exit 1
        fi
    fi  

    mkdir -p ~/Documents/swarm-perception-results/$FILE_NAME

    echo "Saving results to $RESULTS_DIR/$FILE_NAME"

    mv $RESULTS_DIR/*.* $RESULTS_DIR/$FILE_NAME/

fi
