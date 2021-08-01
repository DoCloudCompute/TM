import vec_tools
from STL_reader import read_stl
import numpy as np
import cv2

# first, define screen, generator for rays
def make_screen(screen_distance, resolution):
    corner = (screen_distance, resolution[0]/2, resolution[1]/2) # divide by 2 so that the plane is centered
    edge1 = (0, -1, 0) # on Y axis
    edge2 = (0, 0, -1) # on Z axis

    return corner, edge1, edge2


def gen_ray(pixel_x, pixel_y, screen, viewer_origin):

    # point of intersection
    S = (screen[0][0] + screen[1][0]*pixel_x + screen[2][0]*pixel_y,
         screen[0][1] + screen[1][1]*pixel_x + screen[2][1]*pixel_y,
         screen[0][2] + screen[1][2]*pixel_x + screen[2][2]*pixel_y)

    d = vec_tools.sub3(S, viewer_origin) # directing vector
    d = vec_tools.unit3(d)
    return viewer_origin, d


def gen_triangle_vectors(triangle_vertices, color):
    # color are BGR (Blue, Green, Red)
    # transform triangles to triangles_vec (E1 = B - A | E2 = C - A)
    triangles_vec = []
    for tri in triangle_vertices:
        A = tri[0]
        B = tri[1]
        C = tri[2]
        normal = vec_tools.unit3(tri[3])

        E1 = vec_tools.sub3(B, A)
        E2 = vec_tools.sub3(C, A)

        tri_vec_element = [A, E1, E2, normal, color]
        triangles_vec.append(tri_vec_element)

    return triangles_vec


def gen_bounce_ray(normal, origin, d):
    # https://math.stackexchange.com/questions/13261/how-to-get-a-reflection-vector
    d_dot_n = vec_tools.dot3(d, normal)
    d2 = vec_tools.mult3_scalar(d, 2)
    right_product = vec_tools.mult3_scalar(d2,  d_dot_n)

    bounce_d = vec_tools.sub3(d, right_product)

    return origin, bounce_d


def get_intersection(triangles_vec, ray, max_reflection, reflection_depth = 0):
    pix_RGB = (0,0,0) # set color

    O, d = ray # unpack origin and directing vector

    # will be used to check for closest triangle
    smallest_t = None

    for tri in triangles_vec:
        #print(tri)
        V, E1, E2, normal, color = tri
        VO = vec_tools.sub3(O, V)

        cross1 = vec_tools.cross(E1, E2)
        neg_d = vec_tools.negative(d)
        invdet = -1/(vec_tools.dot3(d, cross1))

        t = abs(vec_tools.dot3(VO, cross1) * invdet)
        if not smallest_t: smallest_t = t

        if t <= smallest_t and t > 0:
            cross2 = vec_tools.cross(VO, neg_d)

            neg_E2 = vec_tools.negative(E2)
            u = vec_tools.dot3(neg_E2, cross2) * invdet
            v = vec_tools.dot3(E1, cross2) * invdet

            if u+v <= 1 and u >= 0 and v >= 0:
                smallest_t = t
                pix_RGB = color

                # if the desired trace is not yet complete, keep reflecting
                if reflection_depth < max_reflection:
                    # origin of bounce ray
                    intersection_coords = (V[0] + E1[0]*u + E2[0]*v,
                                           V[1] + E1[1]*u + E2[1]*v,
                                           V[2] + E1[2]*u + E2[2]*v)

                    bounce_ray = gen_bounce_ray(normal, intersection_coords, d)

                    reflection = get_intersection(triangles_vec, bounce_ray, max_reflection, reflection_depth+1)
                    pix_RGB = vec_tools.add3(pix_RGB, reflection)

    return pix_RGB


def main():
    # first define some constants
    # space is in X,Y,Z where X is the front, Y is sideways and Z is vertical
    viewer_origin = (0,0,0)
    resolution = (720,720)

    # create the image output buffer
    res_image = np.zeros((resolution[1], resolution[0], 3), dtype=np.uint8)

    triangle_vertices = read_stl("../utils/icosphere_red_right.stl")
    triangles_vec = gen_triangle_vectors(triangle_vertices, (0, 0, 255))

    triangle_vertices = read_stl("../utils/icosphere_yellow_left.stl")
    triangles_vec = triangles_vec + gen_triangle_vectors(triangle_vertices, (0, 255, 255))

    screen = make_screen(2330, resolution)

    for pixel_x in range(300, resolution[0]):
        for pixel_y in range(resolution[1]):
            ray = gen_ray(pixel_x, pixel_y, screen, viewer_origin)
            pix_RGB = get_intersection(triangles_vec, ray, max_reflection=1)

            if pix_RGB != (0,0,0):
                res_image[pixel_y, pixel_x] = pix_RGB

        cv2.imshow("wow", res_image)
        cv2.waitKey(1)
        print(pixel_x, end="\r")

    print("done")
    cv2.imshow("wow", res_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

main()
