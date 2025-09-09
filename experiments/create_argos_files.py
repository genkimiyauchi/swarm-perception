import csv
import sys
import os
import re

def main(csv_path, template_path="template.argos"):
    # Read template
    with open(template_path, "r") as f:
        template = f.read()

    # Read CSV
    with open(csv_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            stim_num = row['StimulusNum']
            speed = row['SPEED']
            separation = row['SEPARATION']
            broadcast = row['BROADCAST']

            # Prepare new file content
            content = template
            content = re.sub(r'random_seed="[^"]*"', f'random_seed="{stim_num}"', content)
            content = re.sub(r'max_speed="[^"]*"', f'max_speed="{speed}"', content)
            content = re.sub(r'target_distance_walk="[^"]*"', f'target_distance_walk="{separation}"', content)
            content = re.sub(r'broadcast_duration="[^"]*"', f'broadcast_duration="{broadcast}"', content)

            # Write to new file
            out_filename = f"trial{stim_num}.argos"
            with open(out_filename, "w") as out_file:
                out_file.write(content)
            print(f"Created {out_filename}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python create_argos_files.py input.csv [template.argos]")
        sys.exit(1)
    csv_path = sys.argv[1]
    template_path = sys.argv[2] if len(sys.argv) > 2 else "template.argos"
    main(csv_path, template_path)