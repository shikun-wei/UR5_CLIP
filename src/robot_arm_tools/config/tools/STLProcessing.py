#!/usr/bin/python3

# argv(1) = Path to the new tool CAD file (.stl - named after the new tool - meters)
# argv(2) argv(3) argv(4) argv(5) argv(6) argv(7) = End effector frame position regarding the robot flange frame
# argv(8) = Mass of the new tool

import sys
import os

import trimesh as trimesh
import coacd as coacd

import numpy as np
from scipy.spatial.transform import Rotation as R

print("[STL Processing] Loading mesh")

# Open and load mesh
try:
    toolMesh = trimesh.load_mesh(sys.argv[1])
except:
    print("ERROR WHILE LOADING .STL FILE !")
    sys.exit(1)

# Get flange to end effector transformation
try:
    X, Y, Z, RX, RY, RZ = (
        float(sys.argv[2]),
        float(sys.argv[3]),
        float(sys.argv[4]),
        float(sys.argv[5]),
        float(sys.argv[6]),
        float(sys.argv[7]),
    )
except:
    print("ERROR WHILE LOADING TRANSFORMATION !")
    sys.exit(1)

# Get tool mass
try:
    mass = float(sys.argv[8])
except:
    mass = -1

T = np.eye(4)
T[:3, 3] = np.array([X, Y, Z])
T[:3, :3] = R.from_euler("xyz", [RX, RY, RZ]).as_matrix()

# Rotate and translate the mesh to match the flange frame
toolMesh.apply_transform(T)

print("[STL Processing] Computing inertial properties")

# Evaluate mesh inertial properties
volume = toolMesh.mass_properties["volume"]
centerMass = toolMesh.mass_properties["center_mass"]
inertiaMatrix = toolMesh.mass_properties["inertia"]

if mass != -1:
    inertiaMatrix *= mass / volume
else:
    mass = volume

print("[STL Processing] Creating the .json configuration file")

# Create .json configuration file
toolName = os.path.splitext(os.path.basename(sys.argv[1]))[0]
cwd = os.getcwd()

# if(os.path.exists(cwd+"/"+toolName+"/"+toolName+".json")):
#    print("ERROR WHILE CREATING .JSON FILE - FILE ALREADY EXISTS !")
#    sys.exit(1)


def numpyToStr(array):
    string = "["
    for item in array:
        string += str(np.round(item, 5)) + ","
    string = string[:-1]
    string += "]"
    return string


with open(cwd + "/" + toolName + "/" + toolName + ".json", "w") as configFile:
    configFile.write(
        '{"mass":'
        + str(np.round(mass, 5))
        + ',"centerOfMass":'
        + numpyToStr(centerMass)
        + ',"transformation":'
        + numpyToStr(T.flatten("F"))
        + ',"inertia":'
        + numpyToStr(inertiaMatrix.flatten("F"))
        + ',"collisionModel":{"pointA":[0,0.005,0,0,0,0,0,0,0],"pointB":[0,-0.005,0,0,0,0,0,0,0],"radius":[0.005,0,0]}}'
    )

# Rotate and translate the mesh to match the end effector frame
toolMesh.apply_transform(np.linalg.inv(T))

print("[STL Processing] Retrieving the .xacro description file")

# Find the collision section in the xacro description file
lineIndex = -1

if not os.path.exists(cwd + "/" + toolName + "/" + toolName + ".json"):
    print("ERROR WHILE OPENING .XACRO FILE !")
    sys.exit(1)

with open(cwd + "/" + toolName + "/" + toolName + ".xacro", "r") as descriptionFile:

    contents = descriptionFile.readlines()

for line in contents:
    lineIndex += 1
    if "</visual>" in line:
        lineIndex += 1
        break

if lineIndex < 0:
    print("ERROR WHILE FILLING .XACRO FILE - CANNOT FIND </visual> TAG !")
    sys.exit(1)
elif "<collision>" in contents[lineIndex]:
    print("ERROR WHILE FILLING .JSON FILE - COLLISIONS ALREADY DEFINED !")
    sys.exit(1)

# Create collision volumes : points clustering and volume shape optimization
Nmax = 10
collisionVolumes = []
volumeApproximation = 0

print(
    "[STL Processing] Computing collision volumes - Maximum number of volumes : "
    + str(Nmax)
)

#Remark : Use bounding mesh or convexhull for fine collision mesh !
mesh = coacd.Mesh(toolMesh.vertices, toolMesh.faces)
result = coacd.run_coacd(
    mesh,
    threshold=0.15,
    max_convex_hull=Nmax,
    preprocess_mode="auto",
    preprocess_resolution=50,
    resolution=4000,
    mcts_nodes=20,
    mcts_iterations=150,
    mcts_max_depth=3,
    pca=False,
    merge=True)

for vs, fs in result:
    boundingVolume = trimesh.Trimesh(vs, fs).bounding_primitive
    collisionVolumes.append(boundingVolume)

    volumeApproximation += boundingVolume.volume

print("Found approximation with " + str(len(result)) + " volumes")
print("Initial mesh volume : " + str(np.round(toolMesh.volume, 5)))
print("Approximated volume : " + str(np.round(volumeApproximation, 5)))

collisionVolumes = np.array(collisionVolumes)

mesh_parts = []
for vs, fs in result:
    mesh_parts.append(trimesh.Trimesh(vs, fs))
scene = trimesh.Scene()
np.random.seed(0)
for p in mesh_parts:
    p.visual.vertex_colors[:, :3] = (np.random.rand(3) * 255).astype(np.uint8)
    scene.add_geometry(p)

for c in collisionVolumes:
    c.visual.vertex_colors[:, :4] = np.append(np.random.rand(3) * 255, 50).astype(np.uint8)
    scene.add_geometry(c)
scene.show()

print("[STL Processing] Filling the .xacro description file")

# Create collision volumes description
collisions = ""
for i, volume in enumerate(collisionVolumes):

    r = R.from_matrix(volume.primitive.transform[:3, :3].copy())
    eulerAngles = r.as_euler("xyz")
    origin = '<origin xyz="{0} {1} {2}" rpy="{3} {4} {5}"/>'.format(
        np.round(volume.primitive.transform[0, -1], 3),
        np.round(volume.primitive.transform[1, -1], 3),
        np.round(volume.primitive.transform[2, -1], 3),
        np.round(eulerAngles[0], 3),
        np.round(eulerAngles[1], 3),
        np.round(eulerAngles[2], 3),
    )

    geometry = ""
    if type(volume) is trimesh.primitives.Box:
        geometry = (
            '<box size="${'
            + str(np.round(volume.primitive.extents[0], 3))
            + "+safety_distance} ${"
            + str(np.round(volume.primitive.extents[1], 3))
            + "+safety_distance} ${"
            + str(np.round(volume.primitive.extents[2], 3))
            + '+safety_distance}"/>'
        )
    elif type(volume) is trimesh.primitives.Cylinder:
        geometry = (
            '<cylinder radius="${'
            + str(np.round(volume.primitive.radius, 3))
            + '+safety_distance}" length="${'
            + str(np.round(volume.primitive.height, 3))
            + '+safety_distance}"/>'
        )
    elif type(volume) is trimesh.primitives.Sphere:
        geometry = (
            '<sphere radius="${'
            + str(np.round(volume.primitive.radius, 3))
            + '+safety_distance}"/>'
        )

    collisions += (
        "\t\t\t<collision>\n\t\t\t\t"
        + origin
        + "\n\t\t\t\t"
        + "<geometry>\n\t\t\t\t\t"
        + geometry
        + "\n\t\t\t\t"
        + "</geometry>\n\t\t\t</collision>\n"
    )

    print(
        "[STL Processing] Collision volume "
        + str(i + 1)
        + "/"
        + str(len(collisionVolumes))
    )
    print("\t" + geometry)
    print("\t" + origin)

contents.insert(lineIndex, collisions)

with open(cwd + "/" + toolName + "/" + toolName + ".xacro", "w") as descriptionFile:
    contents = "".join(contents)
    descriptionFile.write(contents)
