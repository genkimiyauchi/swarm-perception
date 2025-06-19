import sys
import cv2
import numpy as np
import os
import os.path
import re  # regular expression
import subprocess  # For running the ffmpeg command

def create_video(dir_path, name, fps, robots, speed, distance, time):
    # Count number of files in directory
    img_num = 0
    frame_pattern = re.compile(r'^frame_\d{10}\.png$')  # Regex for frame_0000000001.png format
    for path in os.listdir(dir_path):
        if os.path.isfile(os.path.join(dir_path, path)) and frame_pattern.match(path):
            img_num += 1

    print('Number of images: {}'.format(img_num))

    # Get shape of image
    img = cv2.imread('{0}frame_0000000001.png'.format(dir_path))
    if img is None:
        print(f"Error: Unable to read the first frame '{dir_path}frame_0000000001.png'. Check the file path or integrity.")
        sys.exit(1)

    frameSize = (img.shape[1], img.shape[0])

    # Construct the output video file name
    # video_filename = f"R{robots}_S{speed}_D{distance}_T{time}.mp4"
    video_filename = f"{name}.mp4"
    video_path = os.path.join(dir_path, video_filename)

    # Writer
    out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4V'), fps, frameSize)

    for i in range(0, img_num):
        print('Frame: {}'.format(i))
        num_zero = 10 - len(str(i))
        prefix = '0' * num_zero
        filename = '{0}frame_{1}{2}.png'.format(dir_path, prefix, i)
        img = cv2.imread(filename)
        if img is not None:
            out.write(img)
        else:
            print(f"Warning: Unable to read frame '{filename}'. Skipping.")
    print(f'Video saved to {video_path}')

    out.release()
    return video_path


def compress_video(video_path, dir_path, robots, speed, distance, time):
    # Construct the compressed video file name
    compressed_video_filename = f"R{robots}_S{speed}_D{distance}_T{time}_compressed.mp4"
    compressed_video_path = os.path.join(dir_path, compressed_video_filename)

    # ffmpeg command
    ffmpeg_command = [
        "ffmpeg", "-i", video_path,
        "-c:v", "libx264", "-preset", "slow", "-crf", "28", "-pix_fmt", "yuv420p",
        compressed_video_path
    ]
    print(f"Running ffmpeg to compress the video: {compressed_video_path}")
    subprocess.run(ffmpeg_command, check=True)
    print(f"Compressed video saved to {compressed_video_path}")


if __name__ == "__main__":
    # Ensure the script is called with the correct number of arguments
    if len(sys.argv) != 7:
        print("Usage: python create_video.py <image-dir-path> <video-name> <robots> <speed> <distance> <time>")
        sys.exit(1)

    # Parse command-line arguments
    robots = sys.argv[3]
    speed = sys.argv[4]
    distance = sys.argv[5]
    time = sys.argv[6]

    # Construct the path dynamically using the parameters
    path = os.path.join(sys.argv[1], f'R{robots}_S{speed}_D{distance}_T{time}/')
    fps = 30

    # Create the video
    video_path = create_video(path, sys.argv[2], fps, robots, speed, distance, time)

    # # Compress the video
    # compress_video(video_path, path, robots, speed, distance, time)