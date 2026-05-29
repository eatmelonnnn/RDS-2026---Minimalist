import pymeshlab

files = {
    'mcp_link.STL': 150000,
    'splay_link.STL': 150000,
    'pip_link.STL': 150000,
}

for f, target in files.items():
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(f)
    original = ms.current_mesh().face_number()
    ms.meshing_decimation_quadric_edge_collapse(targetfacenum=target)
    ms.save_current_mesh(f)
    final = ms.current_mesh().face_number()
    print(f'{f}: {original} → {final} faces')

