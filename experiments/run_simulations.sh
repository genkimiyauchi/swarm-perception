#!/bin/bash

# Path to the target_tracking.argos file

# Get the absolute path of where the script was executed
BASE_DIR="$(pwd)"
ARGOS_FILE="$BASE_DIR/experiments/target_tracking.argos"
TRIAL_IDS_FILE="$BASE_DIR/experiments/trial_ids.txt"
VIDEO_SCRIPT="$BASE_DIR/scripts/create_video.py"

# Read all trial IDs (seeds) into an array
mapfile -t TRIAL_IDS < "$TRIAL_IDS_FILE"
TRIAL_INDEX=0

# Lists of values for each parameter
# QUANTITY_LIST=("5" "10" "20")
# MAX_SPEED_LIST=("5.0" "7.5" "10.0" "12.5" "15.0")
# TARGET_DISTANCE_WALK_LIST=("4" "12" "20" "28" "36")
# BROADCAST_DURATION_LIST=("0" "4" "8" "12" "16")

QUANTITY_LIST=("10")
MAX_SPEED_LIST=("5.0" "7.5" "10.0" "12.5" "15.0")
TARGET_DISTANCE_WALK_LIST=("4" "12" "20" "28" "36")
BROADCAST_DURATION_LIST=("0" "4" "8" "12" "16")

# Loop through all combinations of parameters
for QUANTITY in "${QUANTITY_LIST[@]}"; do
    for MAX_SPEED in "${MAX_SPEED_LIST[@]}"; do
        for TARGET_DISTANCE_WALK in "${TARGET_DISTANCE_WALK_LIST[@]}"; do
            for BROADCAST_DURATION in "${BROADCAST_DURATION_LIST[@]}"; do

                # Get the seed from the trial_ids array
                SEED="${TRIAL_IDS[$TRIAL_INDEX]}"
                ((TRIAL_INDEX++))

                # Construct the frame directory name dynamically
                FRAME_DIRECTORY="$BASE_DIR/frames/R${QUANTITY}_S${MAX_SPEED}_D${TARGET_DISTANCE_WALK}_T${BROADCAST_DURATION}"

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

                # Generate the video
                echo "Generating video..."
                python "$VIDEO_SCRIPT" "$BASE_DIR/frames" "$QUANTITY" "$MAX_SPEED" "$TARGET_DISTANCE_WALK" "$BROADCAST_DURATION"

            done
        done
    done
done