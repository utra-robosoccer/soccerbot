import os

from sklearn.model_selection import train_test_split

# Define paths
base_dir = os.path.expanduser("~/test_data_split")
images_dir = os.path.join(base_dir, "images")
labels_dir = os.path.join(base_dir, "labels")

# Set the desired split ratios
train_ratio = 0.7
val_ratio = 0.15
test_ratio = 0.15

# Create subdirectories for train, val, and test inside images and labels
for split in ["train", "val", "test"]:
    os.makedirs(os.path.join(images_dir, split), exist_ok=True)
    os.makedirs(os.path.join(labels_dir, split), exist_ok=True)

# Gather image and label pairs
image_files = [f for f in os.listdir(images_dir) if f.endswith(".jpg") or f.endswith(".png") and not os.path.isdir(os.path.join(images_dir, f))]
label_files = [f for f in os.listdir(labels_dir) if f.endswith(".txt") and not os.path.isdir(os.path.join(labels_dir, f))]

# Pair images and labels by filename
image_label_pairs = [
    (img, img.replace(".jpg", ".txt").replace(".png", ".txt"))
    for img in image_files
    if img.replace(".jpg", ".txt").replace(".png", ".txt") in label_files
]

# Split pairs into train, validation, and test sets
train_pairs, temp_pairs = train_test_split(image_label_pairs, test_size=(1 - train_ratio), random_state=42)
val_pairs, test_pairs = train_test_split(temp_pairs, test_size=(test_ratio / (test_ratio + val_ratio)), random_state=42)


# Function to move files to their respective split directories
def move_files(pairs, split):
    for img_file, lbl_file in pairs:
        os.rename(os.path.join(images_dir, img_file), os.path.join(images_dir, split, img_file))
        os.rename(os.path.join(labels_dir, lbl_file), os.path.join(labels_dir, split, lbl_file))


# Move files to their destinations
move_files(train_pairs, "train")
move_files(val_pairs, "val")
move_files(test_pairs, "test")

print("Data successfully organized into train, val, and test sets with specified ratios!")
