from time import time
import numpy as np
from PIL import Image
import cv2
import struct

"""
Chapter 0: setting up vars and output
"""

# input
pixels_x = int(1280/1)
pixels_y = int(720/1)
num_triangles = int(1)
fov = int(300/1)

if pixels_x % 2 != 0 or pixels_y % 2 != 0:
    print("ERROR! number of pixels must be even")
    exit(1)

# output
res_image = np.zeros((pixels_y, pixels_x, 3), dtype=np.uint8)

"""
Chapter 1: Setting Up Triangles

First and foremost, read STL files and get triangle verts ( https://en.wikipedia.org/wiki/STL_(file_format) )
Triangle verts (A,B,C) are 3D vectors
In here, the triangle vertices coordinates get converted to vectorial form (A,B,C -> A+u.E1+v.E2).
"""

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

# in the shape of:
"""
triangles = [
            [(Ax, Ay, Az), (Bx, By, Bz), (Cx, Cy, Cz)] # that is one triangle
            ]
"""

# triangle vectors (A+u.E1+v.E2)
# in the shape of:
"""
triangles_vec = [
                [(Ax, Ay, Az), (E1x, E1y, E1z), (E2x, E2y, E2z)] # params of vectorial triangle
                ]
"""
triangles_vec = []


# transform triangles to triangles_vec (E1 = B - A | E2 = C - A)
for i, tri in enumerate(triangles):
    A = (tri[0][0],
         tri[0][1],
         tri[0][2])

    E1 = (tri[1][0]-tri[0][0],
          tri[1][1]-tri[0][1],
          tri[1][2]-tri[0][2])

    E2 = (tri[2][0]-tri[0][0],
          tri[2][1]-tri[0][1],
          tri[2][2]-tri[0][2])

    tri_vec_element = [A, E1, E2]
    triangles_vec.append(tri_vec_element)



"""
Chapter 2: Setting up the view rays

Here, the rays are described by 2 parameters: the origin and the directional vector. The point of origin is common to all rays and represents the point of view.
The horizontal FOV is divided by the number of horizontal pixels to get the angle variation per ray horizontally.
The vertical FOV is calculated by: vertical pixels / horizontal pixels * horizontal FOV. The vertical FOV is then going to be divided by the number
of vertical pixels to get the angle variation per ray vertically. The expression can thus be simplified to: horizontal FOV / horizontal pixels
"""

# ray origin is a point in space (Ox, Oy, Oz) represented by a tuple
ray_origin = (-2,0,0)

# directional vector is a list of tuples: (Mx, My, Mz)
mid_pix_x = int(pixels_x / 2)
mid_pix_y = int(pixels_y / 2)
ray_directions = []
for x in range(0,pixels_x):
    for y in range(0, pixels_y):
        ray_directions.append((fov, x-mid_pix_x, y-mid_pix_y))

"""
Chapter 3: checking for intersections

The beefiest part is here. We are going to recursively check for intersections with those condition: u >= 0; v >= 0; u+v <= 1.
To solve u and v, a triple unknown equation is going to be solved via the determinant.
u and v will give directly the UV coordinates necessary for texture mapping (which will come in v2)

We need to solve:
Ox + t.Mx = Ax + u.E1x + v.E2x
Oy + t.My = Ay + u.E1y + v.E2y
Oz + t.Mz = Az + u.E1z + v.E2z

Reorganizing it gives us:
u.E1x + v.E2x - t.Mx = Ox - Ax
u.E1y + v.E2y - t.My = Oy - Ay
u.E1z + v.E2z - t.Mz = Oz - Az

Therefore, the determinant of the system is:
Det(E1, E2, -M) = - E1x.E2y.Mz - E1z.E2x.My - E1y.E2z.Mx + E1z.E2y.Mx + E1x.E2z.My + E1y.E2x.Mz

To solve u, the numerator is:
Det(O - A, E2, -M) = - (Ox - Ax).E2y.Mz - E2x.My.(Oz - Az) - Mx.(Oy - Ay).E2z + (Oz - Az).E2y.Mx + E2z.My.(Ox - Ax) + Mz.(Oy - Ay).E2x

To solve v, the numerator is:
Det(E1, O - A, -M) = - E1x.(Oy - Ay).Mz - (Ox - Ax).My.E1z - Mx.E1y.(Oz - Az) + E1z.(Oy - Ay).Mx + (Oz - Az).My.E1x + Mz.E1y.(Ox - Ax)

To simplify, O - A is going to be abbreviated to d.


"""

print("Started RTX")

UV_intersects = []
startt = time()

for ray in ray_directions:
    smallest_t = None
    ver_u, ver_v = None, None
    for it, trig in enumerate(triangles_vec):

        d = (ray_origin[0] - trig[0][0],
             ray_origin[1] - trig[0][1],
             ray_origin[2] - trig[0][2])

        cross_det_tv = (d[2]*trig[1][1] - d[1]*trig[1][2],
                        d[0]*trig[1][2] - d[2]*trig[1][0],
                        d[1]*trig[1][0] - d[0]*trig[1][1])

        cross_det_du = (trig[2][1]*ray[2] - trig[2][2]*ray[1],
                        trig[2][2]*ray[0] - trig[2][0]*ray[2],
                        trig[2][0]*ray[1] - trig[2][1]*ray[0])

        main_det = trig[1][0]*cross_det_du[0] + trig[1][1]*cross_det_du[1] + trig[1][2]*cross_det_du[2]

        if round(main_det, 6) != 0:
            inv_det = 1/main_det

            t = (trig[2][0]*cross_det_tv[0] + trig[2][1]*cross_det_tv[1] + trig[2][2]*cross_det_tv[2]) * inv_det

            if t > 0 and (not smallest_t or t < smallest_t):
                v_numerator_det = ray[0]*cross_det_tv[0] + ray[1]*cross_det_tv[1] + ray[2]*cross_det_tv[2]
                v = v_numerator_det * inv_det
                if v >= 0 and v <= 1:
                    u_numerator_det = d[0]*cross_det_du[0] + d[1]*cross_det_du[1] + d[2]*cross_det_du[2]
                    u = u_numerator_det * inv_det

                    if u >= 0 and u+v <= 1:
                        smallest_t = t
                        ver_u = u
                        ver_v = v

    u, v = ver_u, ver_v

    pic_coords = (pixels_x-ray[1]-mid_pix_x-1,
                  pixels_y-ray[2]-mid_pix_y-1)

    if u and v: res_image[pic_coords[1], pic_coords[0]] = [255, u*255, v*255]

    UV_intersects.append((u, v))

endt = time()-startt
print(endt)
print("{:,} rays per second".format(int(len(UV_intersects)/(endt))))

res_image = Image.fromarray(res_image)
res_image.show()
#cv2.imshow("wow", res_image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
