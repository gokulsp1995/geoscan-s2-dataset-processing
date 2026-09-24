import os
import re
from pathlib import Path


def rename_ros2_bags(target_dir):
    """Renames ROS 2 bag folders in target_dir to data_n_ros2 format.

    Examples:
        data_1      -> data_1_ros2
        DataBag_1   -> data_1_ros2
        bag_12      -> data_12_ros2
    """
    target_path = Path(target_dir).resolve()

    if not target_path.exists():
        print(f"Error: Directory '{target_path}' does not exist.")
        return

    # Regex pattern to extract digits from folder names like data_1, DataBag_2, bag_3, etc.
    pattern = re.compile(r"(\d+)")

    renamed_count = 0

    for item in sorted(target_path.iterdir()):
        if item.is_dir():
            # Skip if it is already in the target data_N_ros2 format
            if re.match(r"^data_\d+_ros2$", item.name):
                print(f"Skipping already formatted: {item.name}")
                continue

            match = pattern.search(item.name)
            if match:
                index_num = match.group(1)
                new_name = f"data_{index_num}_ros2"
                new_path = target_path / new_name

                if not new_path.exists():
                    print(f"Renaming: {item.name} -> {new_name}")
                    item.rename(new_path)
                    renamed_count += 1
                else:
                    print(
                        f"Skipping '{item.name}': Target '{new_name}' already exists."
                    )

    print(
        f"\nDone! Successfully renamed {renamed_count} directory/directories."
    )


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        directory = sys.argv[1]
    else:
        # Default fallback to current directory or pass your ros2bag directory path
        directory = input(
            "Enter path to ros2bag directory (or press Enter for current dir): "
        ).strip()
        if not directory:
            directory = "."

    rename_ros2_bags(directory)