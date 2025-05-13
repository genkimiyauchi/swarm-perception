#!/bin/bash

# Path to the target_tracking.argos file
ARGOS_FILE="/home/genki/GIT/swarm-competence/experiments/target_tracking.argos"

# Single seed value
SEED="226"

# Lists of values for each parameter
# QUANTITY_LIST=("5" "10" "20")
# MAX_SPEED_LIST=("6.0" "7.5" "9.0" "10.5" "12.0" "13.5" "15.0")
# TARGET_DISTANCE_WALK_LIST=("5" "10" "15" "20" "25" "30" "35")
# BROADCAST_DURATION_LIST=("0.0" "2.5" "5.0" "7.5" "10.0" "12.5" "15.0")

QUANTITY_LIST=("5")
MAX_SPEED_LIST=("10.5")
TARGET_DISTANCE_WALK_LIST=("20")
BROADCAST_DURATION_LIST=("7.5")

# Loop through all combinations of parameters
for QUANTITY in "${QUANTITY_LIST[@]}"; do
    for MAX_SPEED in "${MAX_SPEED_LIST[@]}"; do
        for TARGET_DISTANCE_WALK in "${TARGET_DISTANCE_WALK_LIST[@]}"; do
            for BROADCAST_DURATION in "${BROADCAST_DURATION_LIST[@]}"; do

                # Construct the frame directory name dynamically
                FRAME_DIRECTORY="frames/R${QUANTITY}_S${MAX_SPEED}_D${TARGET_DISTANCE_WALK}_T${BROADCAST_DURATION}"

                # Ensure the frame directory exists
                if [ ! -d "$FRAME_DIRECTORY" ]; then
                    echo "Frame directory '$FRAME_DIRECTORY' does not exist. Creating it..."
                    mkdir -p "$FRAME_DIRECTORY"
                else
                    # Check if the directory contains any files
                    if [ "$(ls -A "$FRAME_DIRECTORY")" ]; then
                        echo "Frame directory '$FRAME_DIRECTORY' is not empty. Do you want to overwrite its contents? (y/n)"
                        read -r response
                        if [[ "$response" != "y" ]]; then
                            echo "Skipping this simulation."
                            continue
                        fi
                    fi
                fi

                # Update random_seed
                sed -i "s/random_seed=\"[0-9]*\"/random_seed=\"$SEED\"/" "$ARGOS_FILE"

                # Update quantity in the <robots> tag only within the <teams> block
                sed -i "/<teams>/,/<\/teams>/s/quantity=\"[0-9]*\"/quantity=\"$QUANTITY\"/" "$ARGOS_FILE"

                # Update max_speed
                sed -i "s/max_speed=\"[0-9.]*\"/max_speed=\"$MAX_SPEED\"/" "$ARGOS_FILE"

                # Update target_distance_walk
                sed -i "s/target_distance_walk=\"[0-9.]*\"/target_distance_walk=\"$TARGET_DISTANCE_WALK\"/" "$ARGOS_FILE"

                # Update broadcast_duration
                sed -i "s/broadcast_duration=\"[0-9.]*\"/broadcast_duration=\"$BROADCAST_DURATION\"/" "$ARGOS_FILE"

                # Update frame_grabbing directory
                sed -i "s|<frame_grabbing directory=\"[^\"]*\"|<frame_grabbing directory=\"$FRAME_DIRECTORY\"|" "$ARGOS_FILE"

                # Run the simulation
                echo "Running simulation with QUANTITY=$QUANTITY, MAX_SPEED=$MAX_SPEED, TARGET_DISTANCE_WALK=$TARGET_DISTANCE_WALK, BROADCAST_DURATION=$BROADCAST_DURATION"
                argos3 -c "$ARGOS_FILE"

            done
        done
    done
done