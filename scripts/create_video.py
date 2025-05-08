import sys
import cv2
import numpy as np
import os
import os.path
import re  # regular expression

def create_video(dir_path, fps, robots, speed, distance, time):
    # Count number of files in directory
    img_num = 0
    frame_pattern = re.compile(r'^frame_\d{10}\.png$')  # Regex for frame_0000000001.png format
    for path in os.listdir(dir_path):
        if os.path.isfile(os.path.join(dir_path, path)) and frame_pattern.match(path):
            img_num += 1

    print('Number of images: {}'.format(img_num))

    # Get shape of image
    img = cv2.imread('{0}frame_0000000001.png'.format(dir_path))
    frameSize = (img.shape[1], img.shape[0])

    # Construct the output video file name
    video_filename = f"r{robots}_s{speed}_d{distance}_t{time}.mp4"

    # Writer
    out = cv2.VideoWriter(os.path.join(dir_path, video_filename), cv2.VideoWriter_fourcc(*'mp4V'), fps, frameSize)

    for i in range(0, img_num + 1):
        print('Frame: {}'.format(i))
        num_zero = 10 - len(str(i))
        prefix = '0' * num_zero
        filename = '{0}frame_{1}{2}.png'.format(dir_path, prefix, i)
        img = cv2.imread(filename)
        if img is not None:
            out.write(img)
    print(f'Video saved to {os.path.join(dir_path, video_filename)}')

    out.release()


if __name__ == "__main__":
    # Ensure the script is called with the correct number of arguments
    if len(sys.argv) != 5:
        print("Usage: python create_video.py <robots> <speed> <distance> <time>")
        sys.exit(1)

    # Hardcoded path and fps
    path = os.path.join(os.environ['HOME'], 'GIT/swarm-competence/frames/')
    fps = 30

    # Parse command-line arguments
    robots = sys.argv[1]
    speed = sys.argv[2]
    distance = sys.argv[3]
    time = sys.argv[4]

    create_video(path, fps, robots, speed, distance, time)