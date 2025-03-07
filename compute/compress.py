import lz4.frame
import numpy as np
from PIL import Image
import os

def ensure_rgb(image):
    """ Converts any image to 3-channel RGB """
    if image.mode != "RGB":
        image = image.convert("RGB")
    return image

def split_image(image_path):
    """ Splits an image into 4 equal parts """
    img = Image.open(image_path)
    img = ensure_rgb(img)  # Ensure it's in RGB format

    width, height = img.size
    half_width, half_height = width // 2, height // 2

    parts = [
        img.crop((0, 0, half_width, half_height)),  # Top-left
        img.crop((half_width, 0, width, half_height)),  # Top-right
        img.crop((0, half_height, half_width, height)),  # Bottom-left
        img.crop((half_width, half_height, width, height))  # Bottom-right
    ]

    return parts, half_width, half_height  # Return parts and their size

def save_rgb_values(image, output_txt):
    """ Saves pixel RGB values in 'R G B' format per line """
    pixels = np.array(image)
    with open(output_txt, "w") as f:
        for row in pixels:
            for pixel in row:
                f.write(f"{pixel[0]} {pixel[1]} {pixel[2]}\n")

def compress_part(image, part_index):
    """ Compresses an image part using LZ4 """
    img_data = np.array(image, dtype=np.uint8).tobytes()
    compressed_data = lz4.frame.compress(img_data)

    with open(f"part_{part_index}.lz4", "wb") as f:
        f.write(compressed_data)

def main(image_path):
    if not os.path.exists(image_path):
        print("Image file not found!")
        return

    parts, part_width, part_height = split_image(image_path)

    # Save dimensions for reference
    with open("image_dimensions.txt", "w") as f:
        f.write(f"{part_width} {part_height}\n")

    for i, part in enumerate(parts):
        compress_part(part, i)
        save_rgb_values(part, f"part_{i}.txt")

    print("Compression and RGB extraction completed.")

if __name__ == "__main__":
    main("input_image.png")  # Change this to the actual image file
