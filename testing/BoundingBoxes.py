import numpy as np
import STL_reader as stl

class octree():
    def __init__(self, origin, size, parent=True):
        self.origin = origin
        self.size = size
        if parent:
            self.children = subdivide_cube(origin, size)
        else: self.children = []

def subdivide_cube(origin, radius):
    step = radius / 2
    cubes = []

    for x_dir in range(0,3,2):
        x_dir -= 1
        for y_dir in range(0,3,2):
            y_dir -= 1
            for z_dir in range(0,3,2):
                z_dir -= 1

                cube_origin = (origin[0] + step * x_dir,
                               origin[1] + step * y_dir,
                               origin[2] + step * z_dir)
                cubes.append([cube_origin, step])
    return cubes

def make_octree_space(main_cube = [(0,0,0), 10]):
    L1_root = octree(main_cube[0], main_cube[1])
    L1_boxes = L1_root.children
    L2_boxes = [octree(i[0], i[1], parent=False) for i in L1_boxes]
    #L2_boxes = [i.children for i in L2_roots]
    #L3_boxes = [octree(i[0], i[1], parent=False) for i in L2_boxes]

    return L1_boxes, L2_boxes


def fit_octree_space(L1_boxes, L2_boxes, verts):
    for vert in verts:
        for bid, L2_chk in enumerate(L2_boxes):
            x_chk = abs(L2_chk.origin[0]-vert[0]) <= L2_chk.size
            y_chk = abs(L2_chk.origin[1]-vert[1]) <= L2_chk.size
            z_chk = abs(L2_chk.origin[2]-vert[2]) <= L2_chk.size

            if x_chk and y_chk and z_chk:
                print(L2_chk.origin)
                L2_chk.children.append(vert)
                break


ntri = 1
triangles = stl.read_stl("../utils/cube.stl")
L1_boxes, L2_boxes = make_octree_space()
fit_octree_space(L1_boxes, L2_boxes, [(0,0,0)])
print(L2_boxes[0].children)
