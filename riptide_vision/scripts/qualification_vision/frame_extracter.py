# This function will not work nicely with the core dependencies
# This was created using:
# Python 3.5
# Moviepy
from moviepy.editor import VideoFileClip
import math


# Function that extracts frames from video
def extract_frames(videoFile, outputDirectory):
    """Extracts images from video.

    The function creates a .png file for every second of video.

    Parameters
    ----------
    videoFile: str
        The name of the video, it requires the video to be valid.
        Example: './happy_dog.mp4'
    outputDirectory: str
        The name of the directory where the images will be saved, the
        directory is required to be empty.
        Example: './dogs/'
    """
    clip = VideoFileClip(videoFile)
    clipDuration = math.floor(clip.end)

    for i in range(clipDuration):
        clip.save_frame(outputDirectory + 'frame' + str(i) + '.png', t=i)
