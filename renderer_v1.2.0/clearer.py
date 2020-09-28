import struct
import numpy as np
import cv2
from time import time

"""
Step 0: define some operations
"""
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


"""
Step 1: read STL
"""
def read_stl(fname):
    triangles = []

    stl = open(fname, "rb")
    stl_header = stl.read(80)
    stl_n_tri = stl.read(4)
    stl_n_tri = int.from_bytes(stl_n_tri, byteorder="little")

    for face in range(0, stl_n_tri):
        stl_vals = []
        for i in range(0, 12):
            val_bytes = stl.read(4)
            val = float(struct.unpack('<f', val_bytes)[0])
            stl_vals.append(val)

        normal = (stl_vals[0],
                  stl_vals[1],
                  stl_vals[2])

        A = (stl_vals[3],
             stl_vals[4],
             stl_vals[5])

        B = (stl_vals[6],
             stl_vals[7],
             stl_vals[8])

        C = (stl_vals[9],
             stl_vals[10],
             stl_vals[11])

        tri_verts = [A, B, C, normal]
        triangles.append(tri_verts)
        stl.read(2)
    return triangles


"""
Step 2: ignore faces that do not face the screen
"""
def occlude_backface(triangles, screen_edges):
    visible_tris = []
    screen_norm = cross(screen_edges[0], screen_edges[1])

    for tri in triangles:
        tri_norm = tri[3]
        dot_prod = dot3(screen_norm, tri_norm)
        if dot_prod <= 0:
            visible_tris.append(tri[0:3])
    return visible_tris


"""
Step 3: sort tris by distance
"""
def sort_dist(triangles, screen_edges, screen_origin):
    tris_dist = []

    # get cartesian equation of screen plane
    screen_normal = cross(screen_edges[0], screen_edges[1])
    d_offset = dot3(screen_normal, screen_origin)

    for tri in triangles:
        # select smallest
        distance = None
        for vertex in tri:
            test_d = abs(dot3(vertex, screen_normal) + d_offset)
            if not distance or test_d < distance:
                distance = test_d

        tris_dist.append([tri[0], tri[1], tri[2], distance])

    sorted_lst_tmp = sorted(tris_dist, key=lambda elem: elem[3], reverse=True)
    sorted_tris = [elem[0:3] for elem in sorted_lst_tmp]
    return sorted_tris



"""
Step 4: do 2D projection
"""
def vectorize_tris(projected_UV):
    A = (projected_UV[1][0], projected_UV[1][1])
    E1 = vec2(projected_UV[2], A)
    E2 = vec2(projected_UV[3], A)

    return [A, E1, E2]


def project_2d(tri, ray_origin, screen_origin, screen_edges, screen_origin_2d, screen_edges_2d):
    projected_UV = [False]
    for vertex in tri:
        d = vec3(ray_origin, screen_origin)
        ray = vec3(vertex, ray_origin)

        cross_det = cross(screen_edges[1], ray)
        main_det = dot3(screen_edges[0], cross_det)

        if round(main_det, 6) != 0:
            inv_det = 1/main_det

            u_numerator_det = dot3(d, cross_det)
            u = u_numerator_det * inv_det

            v_numerator_det = det3(screen_edges[0], d, ray)
            v = v_numerator_det * inv_det

            UV_coords = (screen_edges_2d[0][0]*u + screen_edges_2d[1][0]*v + pixels_x,
                         screen_edges_2d[0][1]*u + screen_edges_2d[1][1]*v + pixels_y)

            projected_UV.append(UV_coords)
            if u >= 0 and u <= 1 and v >= 0 and v <= 1:
                projected_UV[0] = True

    if projected_UV[0]:
        return vectorize_tris(projected_UV)
    else:
        return None


"""
Step 5: ray trace
"""
def draw_tris(triangles_vec, res_image, pixels_x, pixels_y):
    global rps_avg_lst

    for tri_id, tri in enumerate(triangles_vec):
        x_origin = int(tri[0][0])
        y_origin = int(tri[0][1])

        if tri[1][0] > 0: E1x = int(tri[1][0]+0.5)
        else: E1x = int(tri[1][0]-0.5)

        if tri[1][1] > 0: E1y = int(tri[1][1]+0.5)
        else: E1y = int(tri[1][1]-0.5)

        if tri[2][0] > 0: E2x = int(tri[2][0]+0.5)
        else: E2x = int(tri[2][0]-0.5)

        if tri[2][1] > 0: E2y = int(tri[2][1]+0.5)
        else: E2y = int(tri[2][1]-0.5)

        if not ((E1x == 0 and E1y == 0) or (E2x == 0 and E2y == 0)):
            delta_u = 1/(abs(E1x) + abs(E1y))
            delta_v = 1/(abs(E2x) + abs(E2y))
            u, v, i = 0, 0, 0
            ray_start = time()

            while u <= 1:
                v = 0
                while v <= 1 and u+v <= 1:
                    pic_coords = (int(E1x*u + E2x*v + x_origin),
                                  int(E1y*u + E2y*v + y_origin))

                    # verify that pic_coords is within frame
                    if 0 <= pic_coords[0] < pixels_x and 0 <= pic_coords[1] < pixels_y:
                        # verify that the pixels have not already been drawn
                        if not res_image[pic_coords[1], pic_coords[0]].any():
                            # dat shit be very time consuming
                            res_image[pic_coords[1], pic_coords[0]] = [255, u*255, v*255]

                    i += 1 # rays traced

                    v += delta_v
                u += delta_u

            rps = (i)/(time()-ray_start)
            rps_avg_lst.append(rps)
            if len(rps_avg_lst) % 10 == 0: print("{:,} rays per sec".format(int(rps)))
        #cv2.imshow("wow", res_image)
        #cv2.waitKey(300)

    return res_image


"""
main loop
"""

# constants
pixels_x = int(720/1)
pixels_y = int(720/1)
res_image = np.zeros((pixels_y, pixels_x, 3), dtype=np.uint8)

ray_origin = (-3, 0, 0)
screen_origin = (-1,-1,-1)
screen_edges = [(0, 2, 0), (0, 0, 2)]

screen_origin_2d = (0.5 * pixels_x,
                    0.5 * pixels_y)

screen_edges_2d = [(-pixels_x, 0), (0, -pixels_y)]


# start real stuff
rps_avg_lst = []
startt = time()

triangles = read_stl("../renderer_v1.1.0/mesh.stl")
occluded_tris = occlude_backface(triangles, screen_edges)
sorted_triangles = sort_dist(occluded_tris, screen_edges, screen_origin)

# dat could be multithreaded on CPU
triangles_vec = []
for tri in sorted_triangles:
    tri_vec = project_2d(tri, ray_origin, screen_origin, screen_edges, screen_origin_2d, screen_edges_2d)

    if tri_vec:
        triangles_vec.append(tri_vec)

# dat shit has to be single thread
res_image = draw_tris(triangles_vec, res_image, pixels_x, pixels_y)

# print stats
print("")
endt = round(time() - startt, 3)
print("Took", endt, "seconds.")

rps_avg = sum(rps_avg_lst)/len(rps_avg_lst)
max_rps = max(rps_avg_lst)
min_rps = min(rps_avg_lst)
print("Average RPS: {:,}".format(int(rps_avg)))
print("Minimum RPS: {:,}".format(int(max_rps)))
print("Maximum RPS: {:,}".format(int(min_rps)))

# display image
cv2.imshow("wow", res_image)
cv2.waitKey(0)
