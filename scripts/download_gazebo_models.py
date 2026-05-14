#!/usr/bin/env python3
import os
import subprocess
import sys

def main():
    models_dir = os.path.expanduser("~/.gazebo/models")
    os.makedirs(models_dir, exist_ok=True)
    
    models = ["coke_can", "bowl", "cricket_ball", "wood_cube_10cm", "monkey_wrench"]
    repo_url = "https://github.com/osrf/gazebo_models/trunk"
    
    # Check if svn is installed
    try:
        subprocess.run(["svn", "--version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("svn (Subversion) could not be found. Installing...")
        try:
            subprocess.run(["sudo", "apt-get", "update"], check=True)
            subprocess.run(["sudo", "apt-get", "install", "-y", "subversion"], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Failed to install svn. Please install it manually: sudo apt-get install subversion\nError: {e}")
            sys.exit(1)
            
    print(f"Downloading Gazebo models to {models_dir}...")
    
    # Change working directory to models_dir
    os.chdir(models_dir)
    
    for model in models:
        if os.path.isdir(model):
            print(f"[✓] Model '{model}' already exists. Skipping.")
        else:
            print(f"[ ] Downloading '{model}'...")
            try:
                subprocess.run(["svn", "export", f"{repo_url}/{model}"], check=True)
            except subprocess.CalledProcessError as e:
                print(f"Failed to download {model}: {e}")
                
    print("✅ All models processed successfully!")

if __name__ == "__main__":
    main()
