import sys
import numpy as np
from PIL import Image

def reconstruct_image(width, height, txt_file, output_image):
    try:
        with open(txt_file, 'r') as f:
            lines = [line.strip().split() for line in f]

        # Determine if the input is grayscale (1 value) or RGB (3 values)
        if len(lines[0]) == 3:
            mode = "RGB"
            data = np.array([[int(v) for v in line] for line in lines], dtype=np.uint8)
            data = data.reshape((height, width, 3))  # Reshape into (H, W, 3)
        elif len(lines[0]) == 1:
            mode = "L"  # Grayscale mode
            data = np.array([int(line[0]) for line in lines], dtype=np.uint8)
            data = data.reshape((height, width))  # Reshape into (H, W)
        else:
            print("Error: Unexpected number of values per line in the text file.")
            return

        # Create and save the image
        img = Image.fromarray(data, mode)
        img.save(output_image)
        print(f"✅ Image successfully reconstructed and saved as {output_image}")

    except FileNotFoundError:
        print("Error: Input text file not found.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python reconstruct_image.py <width> <height> <input.txt> <output.png>")
    else:
        width, height = int(sys.argv[1]), int(sys.argv[2])
        txt_file = sys.argv[3]
        output_image = sys.argv[4]
        reconstruct_image(width, height, txt_file, output_image)
