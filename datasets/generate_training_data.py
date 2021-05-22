import cv2
import glob
import argparse
from os.path import join


class ImageGenerator:
    """
    Class. Provides image generation methods from given paths.
    """

    def __init__(self, input_directory, output_directory, number_of_frames):
        self.video_file_paths = glob.glob(join(input_directory, '*.mp4'))
        # Gets every video from input directory.

        self.output_directory = output_directory
        self.frame_multiplier = int(number_of_frames)

    def generate_images_from_videos(self):
        output_count = 0  # Set output file count so all images are unique.
        for video_file_path in self.video_file_paths:

            print(f"Generating dataset from {video_file_path}.")
            cv_video_capture = cv2.VideoCapture(video_file_path)

            video_frame_counter = 0

            while cv_video_capture.isOpened():
                success, image = cv_video_capture.read()
                print(
                    f'Frame {video_frame_counter} from {video_file_path}.')

                if success:  # Check if the video has been loaded correctly.
                    cv2.imwrite(join(
                        self.output_directory,
                        f"frame{output_count}.jpg"),
                        image)
                    # Save frame as JPEG file.

                    output_count += 1
                    video_frame_counter += 1 * self.frame_multiplier
                    cv_video_capture.set(1, video_frame_counter)
                else:
                    break


if __name__ == "__main__":
    """
    Example Usage:

    python3 generate_training_data.py -i test/videos/ -o test/ -f 10
    """
    args = argparse.ArgumentParser()
    args.add_argument("-i", "--input", help="Path to video directory.")
    args.add_argument("-o", "--output", help="Path to image directory.")
    args.add_argument("-f", "--frames", help="Frame multiplier.")

    args = args.parse_args()
    image_generator = ImageGenerator(args.input, args.output, args.frames)
    image_generator.generate_images_from_videos()
