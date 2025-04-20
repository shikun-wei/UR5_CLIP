#!/usr/bin/python3

import sys
import os

from scipy.spatial.transform import Rotation as R
import numpy as np

import trimesh as trimesh

print("[STL Transformation] Loading mesh")

#Open and load mesh
try:
    fileName = sys.argv[1]
    toolMesh = trimesh.load_mesh(fileName)
except:
    print("ERROR WHILE LOADING .STL FILE !")
    sys.exit(1)

#Get transformation type
try:
    transformation = sys.argv[2]
except:
    print("ERROR WHILE LOADING TRANSFORMATION TYPE !")
    sys.exit(1)

if(transformation != "translation" and transformation != "rotation"):
    print("INVALID TRANSFORMATION TYPE !")
    sys.exit(1)

#Get transformation parameters
try:
    X,Y,Z = float(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5])
except:
    print("ERROR WHILE LOADING TRANSFORMATION PARAMETERS !")
    sys.exit(1)

T = np.eye(4)
if(transformation == "translation"):
    T[:3,3] = np.array([X,Y,Z])
elif(transformation == "rotation"):
    T[:3,:3] = R.from_euler("xyz",[X,Y,Z]).as_matrix()

#Apply transformation to the mesh
print("[STL Transformation] Applying transformation")
toolMesh.apply_transform(T)

scene = trimesh.Scene()

scene.add_geometry(toolMesh)

scale = np.linalg.norm(toolMesh.extents)
axis = trimesh.creation.axis(origin_size=scale/75,axis_radius=scale/75,axis_length=scale/5)
scene.add_geometry(axis)

scene.show()

save = input("[STL Transformation] Save mesh ? y/n")

if(save == "y" or save == "Y"):
    print("[STL Transformation] Saving mesh")
    overwrite = input("[STL Transformation] Overwrite STL file ? y/n")
    if(overwrite != "y" and overwrite != "Y"):
        splitPath = os.path.split(fileName)
        fileName = splitPath[0] + "/transformed_" + splitPath[1]
    toolMesh.export(fileName)
