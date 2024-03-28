from PIL import Image
import numpy as np
import yaml
import os

# Load the image file
file_path = '/home/hirayama-d/research_ws/src/line_detection/scripts/transformed_image.png'  # Change this to your actual file path：input.png
img = Image.open(file_path)
img = img.rotate(-90, expand=True)

# Convert the image to grayscale
gray_img = img.convert('L')

# Prepare the mapping from white to black (0 in PGM) and brown to gray
# Assuming brown is not a pure brown and can be approximated, we define a threshold
def map_colors(pixel):
    if pixel == 255:  # White to black
        return 0
    else:  # Other colors to gray
        return 254  # The gray value might need adjustment

# Apply the mapping
mapped_img = gray_img.point(map_colors)
# ⇛ここが何しているのか分からない

# Save the mapped image as PGM
pgm_file_path = 'mapped_sample.pgm'
mapped_img.save(pgm_file_path, format='PPM')

# Create the YAML file content
yaml_content = {
    'image': pgm_file_path,
    'resolution': 0.01,  # Assuming a resolution of 0.01m/pixel
    'origin': [0.0, 361, 0.0],  # Assuming the origin is at the bottom-left corner
    'occupied_thresh': 0.65,  # Threshold above which the space is considered occupied
    'free_thresh': 0.196,  # Threshold below which the space is considered free
    'negate': 0  # Whether to negate the grayscale values in the image
}

# Save the YAML content to a file
yaml_file_path = 'map_sample.yaml'
with open(yaml_file_path, 'w') as yaml_file:
    yaml.dump(yaml_content, yaml_file, default_flow_style=False)

# Now you have `pgm_file_path` and `yaml_file_path` which are paths to the generated files.
