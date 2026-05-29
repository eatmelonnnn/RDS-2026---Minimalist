### Opening mujoco viewer
1. Ensure you are in the folder that contains finger.xml
2. python3 -m mujoco.viewer --mjcf=finger.xml

## Run virtual environmnent
source /mujoco_env/bin/activate     # have to create in that directory first

## Check STL number of faces
python3 -c "
from stl import mesh
import os

for f in os.listdir('.'):
    if f.endswith('.STL') or f.endswith('.stl'):
        m = mesh.Mesh.from_file(f)
        print(f'{f}: {len(m.vectors)} faces')
"

