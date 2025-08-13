import os
import glob

def reset_snapshots():
    snapshots_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'snapshots')
    files_to_remove = glob.glob(os.path.join(snapshots_dir, '*'))
    for file_path in files_to_remove:
        if os.path.isfile(file_path):
            os.remove(file_path)
            print(f"Removed: {file_path}")
    print(f"Cleared {len([f for f in files_to_remove if os.path.isfile(f)])} files from snapshots directory")