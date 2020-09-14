from math import sqrt
import numpy as np
from PIL import Image
import cv2
from time import time
import struct

"""
The idea is to take triangle vertices and translate them to 2D surface UV
"""

pixels_x = int(720/1)
pixels_y = int(720/1)
res_image = np.zeros((pixels_y,pixels_x,3), dtype=np.uint8)

triangles = []

stl = open("mesh.stl", "rb")
stl_header = stl.read(80)
stl_n_tri = stl.read(4)
stl_n_tri = int.from_bytes(stl_n_tri, byteorder="little")

for face in range(0, stl_n_tri):
    stl_vals = []
    for i in range(0,12):
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

    tri_verts = [A, B, C]
    triangles.append(tri_verts)

    stl.read(2)

triangles_vec = []
ray_origin = (-3, 0, 0)
screen_origin = (-1,-1,-1)
screen_edges = [(0, 2, 0), (0, 0, 2)]

screen_origin_2d = (0.5 * pixels_x,
                    0.5 * pixels_y)

screen_edges_2d = [(-pixels_x, 0), (0, -pixels_y)]

# project all triangle verts to screen

startt = time()

for tri in triangles:
    projected_UV = [False]
    for vertex in tri:
        d = (ray_origin[0] - screen_origin[0],
             ray_origin[1] - screen_origin[1],
             ray_origin[2] - screen_origin[2])

        ray = (vertex[0] - ray_origin[0],
               vertex[1] - ray_origin[1],
               vertex[2] - ray_origin[2])

        cross_det = (screen_edges[1][1]*ray[2] - screen_edges[1][2]*ray[1],
                     screen_edges[1][2]*ray[0] - screen_edges[1][0]*ray[2],
                     screen_edges[1][0]*ray[1] - screen_edges[1][1]*ray[0])

        main_det = screen_edges[0][0]*cross_det[0] + screen_edges[0][1]*cross_det[1] + screen_edges[0][2]*cross_det[2]

        if round(main_det, 6) != 0:
            inv_det = 1/main_det

            u_numerator_det = d[0]*cross_det[0] + d[1]*cross_det[1] + d[2]*cross_det[2]
            u = u_numerator_det * inv_det

            v_numerator_det = screen_edges[0][0]*d[1]*ray[2] + screen_edges[0][2]*d[0]*ray[1] + screen_edges[0][1]*d[2]*ray[0] - screen_edges[0][2]*d[1]*ray[0] - screen_edges[0][0]*d[2]*ray[1] - screen_edges[0][1]*d[0]*ray[2]
            v = v_numerator_det * inv_det

            UV_coords = (screen_edges_2d[0][0]*u + screen_edges_2d[1][0]*v + pixels_x,
                         screen_edges_2d[0][1]*u + screen_edges_2d[1][1]*v + pixels_y)

            projected_UV.append(UV_coords)

            if u >= 0 and u <= 1 and v >= 0 and v <= 1: projected_UV[0] = True

    if projected_UV[0]:
        # the triangle is visible, vectorize it in 2d
        A = (projected_UV[1][0],
             projected_UV[1][1])

        E1 = (projected_UV[2][0] - A[0],
              projected_UV[2][1] - A[1])

        E2 = (projected_UV[3][0] - A[0],
              projected_UV[3][1] - A[1])

        tri_vec_element = [A, E1, E2]
        triangles_vec.append(tri_vec_element)

rps_avg_lst = []
precalc_time = []

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

                if not (pic_coords[0] < 0 or pic_coords[1] < 0 or pic_coords[0] > pixels_x or pic_coords[1] > pixels_y):
                    # dat shit be very time consuming
                    res_image[pic_coords[1], pic_coords[0]] = (255, u*255, v*255)

                i += 1 # rays traced

                v += delta_v
            u += delta_u

        rps = (i)/(time()-ray_start)
        rps_avg_lst.append(rps)
        if len(rps_avg_lst) % 10 == 0: print("{:,} rays per sec".format(int(rps)))

print("")
endt = round(time() - startt, 3)
print("Took", endt, "seconds.")

rps_avg = sum(rps_avg_lst)/len(rps_avg_lst)
max_rps = max(rps_avg_lst)
min_rps = min(rps_avg_lst)
print("Average RPS: {:,}".format(int(rps_avg)))
print("Minimum RPS: {:,}".format(int(max_rps)))
print("Maximum RPS: {:,}".format(int(min_rps)))
print("Time spent in precalc:", sum(precalc_time))

cv2.imshow("wow", res_image)
cv2.waitKey(0)
