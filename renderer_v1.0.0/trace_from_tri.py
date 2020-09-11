from math import sqrt
import numpy as np
from PIL import Image
import cv2
from time import time

"""
The idea is to take triangle vertices and translate them to 2D surface UV
"""

pixels_x = 720
pixels_y = 720
res_image = np.zeros((pixels_y,pixels_x,3), dtype=np.uint8)

triangles = [
[(3,1,0), (3,-1,0), (3,1,-1)],
]

triangles_vec = []

ray_origin = (0, 0, 0)

screen_origin = (2,-1,-1)
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

        #pic_coords = (screen_edges_2d[0][0]*u + screen_origin_2d[0],
        #              screen_edges_2d[1][1]*v + screen_origin_2d[1])
        #print(pic_coords)

for tri in triangles_vec:
    x_origin = tri[0][0]
    y_origin = tri[0][1]

    if round(x_origin, 6) == round(x_origin, 0):
        x_origin = round(x_origin, 0)
    if round(y_origin, 6) == round(y_origin, 0):
        y_origin = round(y_origin, 0)

    x_offset = x_origin - round(x_origin, 0)
    y_offset = y_origin - round(y_origin, 0)

    delta_u = 1/sqrt(tri[1][0]**2 + tri[1][1]**2)
    delta_v = 1/sqrt(tri[2][0]**2 + tri[2][1]**2)

    u, v = 0, 0

    while u <= 1:
        v = 0
        while v <= 1 and u+v <= 1:
            pic_coords = (tri[1][0]*u + tri[2][0]*v + x_origin,
                          tri[1][1]*u + tri[2][1]*v + y_origin)

            res_image[int(pic_coords[1]), int(pic_coords[0])] = [255, u*255, v*255]

            v += delta_v
        u += delta_u

print("done")
endt = time() - startt
print(endt)
#res_image = Image.fromarray(res_image)
#res_image.show()
cv2.imshow("wow", res_image)
cv2.waitKey(0)
