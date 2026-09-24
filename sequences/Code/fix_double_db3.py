#!/usr/bin/env python3
import sys
from pathlib import Path

def clean_ros2bag_structure(target_dir):
    base = Path(target_dir).resolve()
    print(f"Cleaning folder structure in: {base}")

    # 1. Fix .db3.db3 files and internal metadata references
    for bad_file in list(base.rglob("*.db3.db3")):
        parent = bad_file.parent
        corrected_name = bad_file.name[:-4]  # removes extra '.db3'
        corrected_file = parent / corrected_name
        
        print(f"Renaming file: {bad_file.name} -> {corrected_name}")
        bad_file.rename(corrected_file)

        meta_file = parent / "metadata.yaml"
        if meta_file.exists():
            content = meta_file.read_text()
            if bad_file.name in content:
                meta_file.write_text(content.replace(bad_file.name, corrected_name))
                print(f"  Updated: {meta_file.name}")

    # 2. Rename outer folders (remove .db3 from folder name)
    for folder in list(base.glob("*.db3")):
        if folder.is_dir():
            clean_folder_name = folder.name[:-4]  # e.g. data_0.db3 -> data_0
            new_folder_path = folder.parent / clean_folder_name
            if not new_folder_path.exists():
                print(f"Renaming folder: {folder.name} -> {clean_folder_name}")
                folder.rename(new_folder_path)

    print("Done! Folder structure cleaned.")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "."
    clean_ros2bag_structure(path)