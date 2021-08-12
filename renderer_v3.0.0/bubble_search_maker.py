import vec_tools
from STL_reader import read_stl

def step1(triangles):
    """
    this shit will make an average of the location of the vertices and find the center or something
    its called a centroid
    """
    for face in triangles:
        A = face[0]
        B = face[1]
        C = face[2]

        # average out these coords
        # find vertex farthest from center

triangles = read_stl("STLs/refl_plane.stl")
