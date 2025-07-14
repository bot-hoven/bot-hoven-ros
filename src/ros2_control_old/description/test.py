from glob import glob

mesh_files = glob('meshes/*')
print(f"Found mesh files: {mesh_files}")