"""
The idea is to make triangles bounding boxes
"""
import numpy as np
from random import uniform
from time import time

def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])

def det3(a, b, c):
    return a[0]*b[1]*c[2] + b[0]*c[1]*a[2] + c[0]*a[1]*b[2] - c[0]*b[1]*a[2] - b[0]*a[1]*c[2] - a[0]*c[1]*b[2]

def det2(a, b):
    return a[0]*b[1] - b[0]*a[1]

def dot3(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def dot2(a, b):
    return a[0]*b[0] + a[1]*b[1]

def vec3(a, b):
    return (a[0] - b[0],
            a[1] - b[1],
            a[2] - b[2])

def vec2(a, b):
    return (a[0] - b[0],
            a[1] - b[1])

def vec3_norm(a):
    return (a[0]**2 + a[1]**2 + a[2]**2)**0.5

def unit_vec3(a):
    return (a[0]/vec3_norm(a),
            a[1]/vec3_norm(a),
            a[2]/vec3_norm(a))

def make_random_triangles(ntri = 1):
    verts = []
    for n in range(ntri):
        verts.append([(uniform(-50,50), uniform(-50,50), uniform(-50,50)) for i in range(3)])
    return verts




def angle_BVH(triangles, nelem):
    ntri = len(triangles)
    angles = np.zeros((nelem+1))

    for tri_id, tri in enumerate(triangles):
        median_x = (tri[0][0]+tri[1][0]+tri[2][0])/3
        median_y = (tri[0][1]+tri[1][1]+tri[2][1])/3
        median_z = (tri[0][2]+tri[1][2]+tri[2][2])/3

        median = unit_vec3((median_x, median_y, median_z))
        relative_vec = (1, 0, 0)

        angle = dot3(median, relative_vec)

        BVH_idx = 0.5*(angle+1) * nelem
        BVH_idx = int(BVH_idx+0.5)
        angles[BVH_idx] = tri_id+1

    return angles

def reflection_intersect(ray, triangles, angles, nelem):
    unit_ray = unit_vec3(ray)
    ray_angle = dot3(unit_ray, (1, 0, 0))

    ray_BVH = int(0.5*(ray_angle+1) * nelem)

    print(ray_BVH)

    tri_hit = angles[ray_BVH]
    i = 0
    chk_idx_up = ray_BVH
    chk_idx_down = ray_BVH

    while tri_hit == 0:
        i += 1
        if ray_BVH+i < nelem:
            tri_hit = angles[ray_BVH+i]
        if tri_hit == 0 and ray_BVH-i > 0:
            tri_hit = angles[ray_BVH-i]
        elif ray_BVH+i >= nelem:

            break


        chk_idx_down -= i
        angle_val_up = angles[chk_idx_up]
        angle_val_down = angles[chk_idx_up]
        print(ray_BVH)

    return ray_BVH

startt = time()
nelem = 2**16
triangles = make_random_triangles(100)
#triangles = [[(2, -1, -1), (2, -1, 1), (2, 1, 0)]]
ray = (1, 0, 0)
angles = angle_BVH(triangles, nelem)

frame_start = time()
print(reflection_intersect(ray, triangles, angles, nelem))
endt = time()

print("Total time:", endt-startt)
print("Frame time:", endt-frame_start)
