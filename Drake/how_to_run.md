# Minimalist Finger Model
This project runs a Drake-based finger model visualization on Linux.

## Setup Instructions

### 1. Install Drake and required dependencies
Make sure Drake and its dependencies are installed on your Linux system.  
You can also use an online platform that supports Drake if you prefer not to install it locally.

### 2. Download the required project folders
Download the following folders from the repository:

- `minimalist finger objs`
- `minimalist finger sdfs`

### 3. Update mesh paths inside the SDF files
Each `.sdf` file contains `<uri>` entries that point to the `.obj` mesh files.

Open every SDF file in the `minimalist finger sdfs` folder and update the paths inside each `<uri>` tag so they match the local path where your `minimalist finger objs` folder is stored.

For example, change:
<uri>old/path/to/file.obj</uri> to: <uri>/home/your_username/path_to_project/minimalist finger objs/file.obj</uri>

### 4. Run the python file
python3 finger_model.py
