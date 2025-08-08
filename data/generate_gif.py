from PIL import Image
import os
import imageio


def png_to_gif(input_folder, output_gif, duration=0.5, resize=None, loop_delay=0):
    # Get all PNG files in the input folder
    png_files = [f for f in os.listdir(input_folder) if f.endswith('.png')]

    # Sort files by their numeric index in the filename
    png_files.sort(key=lambda x: int(os.path.splitext(x)[0]))

    # Read all images
    images = []
    for filename in png_files:
        file_path = os.path.join(input_folder, filename)
        img = Image.open(file_path)
        if resize:
            img = img.resize(resize)
        images.append(img)

     # Add a copy of the last frame with the loop delay
    if loop_delay > 0 and len(images) > 0:
        last_frame = images[-1]
        images.extend([last_frame] * int(loop_delay / duration))

    # Save images as GIF
    imageio.mimsave(output_gif, images, duration=duration, loop=0)


if __name__ == '__main__':
    # Parameters
    input_folder = './png'  # Path to the folder containing PNG files
    output_gif = 'output.gif'  # Name of the output GIF file
    duration = 0.001  # Duration of each frame in seconds
    resize = None  # Resize dimensions (optional)
    loop_delay = 2.0  # Delay after last frame before looping (seconds)

    # Execute the function
    png_to_gif(input_folder, output_gif, duration=duration,
               resize=resize, loop_delay=loop_delay)
    print(f"GIF file '{output_gif}' has been created successfully!")
