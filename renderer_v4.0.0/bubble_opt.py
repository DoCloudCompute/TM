import vec_tools
from STL_reader import read_stl
from bubble_search_v2 import make_tree, find_farthest_val
import numpy as np
import cv2
import time
from math import acos, asin, atan, degrees

"""
Globals
"""
hdri_img = cv2.imread("HDRIs/carpentry_shop_02_4k.png", 1)
reflection_count = 0


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

        centroid = vec_tools.centroid3(A, B, C)

        radius = find_farthest_val([A, B, C], centroid)

        E1 = vec_tools.sub3(B, A)
        E2 = vec_tools.sub3(C, A)

        normal = vec_tools.cross(E1, E2)

        tri_vec_element = [A, E1, E2, normal, color]
        tri_bubble = [centroid, radius, tri_vec_element]
        triangles_vec.append(tri_bubble)

    return triangles_vec


def gen_bounce_ray(normal, origin, d):
    # https://math.stackexchange.com/questions/13261/how-to-get-a-reflection-vector
    normal = vec_tools.unit3(normal)
    d_dot_n = vec_tools.dot3(d, normal)
    normal2 = vec_tools.mult3_scalar(normal, 2)
    right_product = vec_tools.mult3_scalar(normal2,  d_dot_n)

    bounce_d = vec_tools.sub3(d, right_product)

    return origin, bounce_d


def get_HDRI(ray):
    global hdri_img
    """
    World sphere has its center where the ray originates from
    First see where in XYZ does the ray intersect with the world sphere
    since the ray is centered on the sphere, the directing vector of the ray can be taken as intersection point
    then calculate the X angle and the Y angle
    divide both angles by 360° then multiply by HDRI resolution and that should get us the UV coordinates
    for the 2D texture
    """

    x_res = hdri_img.shape[1]
    y_res = hdri_img.shape[0]

    O, d = ray
    d = vec_tools.unit3(d)

    x, y, z = (d[0],
               d[1],
               d[2])

    sign_of_y = 1
    if y < 0: sign_of_y = -1

    divider = vec_tools.norm2((x, y))
    if divider == 0:
        cos_gamma = 1
    else:
        cos_gamma = x / divider

    sin_theta = z # here the division by r is not needed as r would be the length of d but d is a unit vector

    gamma = degrees(acos(cos_gamma)) * sign_of_y + 180
    theta = degrees(asin(sin_theta)) + 90 # as theta is between -90° and 90°

    u = (gamma / 360) * x_res
    v = (theta / 180) * y_res

    u = x_res - int(u) - 1
    v = y_res - int(v) - 1

    return tuple(hdri_img[v, u])


def walk_tree(bubble_tree, ray):
    """
    first check the number of elements in list, if theres 3 of em, its a bubble
    otherwise, its a triangle so return just that

    get 2 first list elems, see if intersect or no
    if yes, call again for each sub element
    """

    if len(bubble_tree[0]) != 3:
        triangle = bubble_tree
        return triangle

    O, d = ray
    centroid, radius_squared = bubble_tree[0:2]

    # CRM page 64
    OC = vec_tools.sub3(centroid, O)
    OC_cross_d = vec_tools.cross(OC, d)
    dist_sq = vec_tools.norm3_sq(OC_cross_d)

    if dist_sq <= radius_squared:
        potential_intersections = []
        groups = bubble_tree[2]

        for group in groups:
            res = walk_tree(group, ray)
            if res != None:
                potential_intersections += res
    else:
        return

    return potential_intersections


def get_intersection(bubble_tree, ray, max_reflection, reflection_depth = 0):
    global reflection_count

    triangles_vec = walk_tree(bubble_tree, ray)

    if triangles_vec == None:
        return get_HDRI(ray)

    pix_RGB = None # set color

    intersected_tri = None

    O, d = ray # unpack origin and directing vector

    # will be used to check for closest triangle
    smallest_t = None

    for tri in triangles_vec:
        V, E1, E2, normal, color = tri

        # backface culling
        if vec_tools.dot3(d, normal) >= 0:
            continue

        VO = vec_tools.sub3(O, V)

        cross1 = normal
        neg_d = vec_tools.negative(d)
        pre_invdet = vec_tools.dot3(d, cross1)

        if abs(pre_invdet) < 1e-8:
            continue

        invdet = -1/pre_invdet

        t = vec_tools.dot3(VO, cross1) * invdet
        if t < 1e-6:
            continue

        if smallest_t == None or t < smallest_t:
            cross2 = vec_tools.cross(VO, neg_d)

            neg_E2 = vec_tools.negative(E2)
            u = vec_tools.dot3(neg_E2, cross2) * invdet
            if u > 1 or u < 0:
                continue

            v = vec_tools.dot3(E1, cross2) * invdet

            if u+v <= 1 and u >= 0 and v >= 0:
                smallest_t = t
                pix_RGB = color
                intersected_tri = tri
                intersected_u = u
                intersected_v = v

    if intersected_tri == None:
        pix_RGB = get_HDRI(ray)

    # if the desired trace is not yet complete, keep reflecting
    if reflection_depth < max_reflection and intersected_tri != None:
        # origin of bounce ray
        V, E1, E2, normal, color = intersected_tri
        u = intersected_u
        v = intersected_v
        intersection_coords = (V[0] + E1[0]*u + E2[0]*v,
                               V[1] + E1[1]*u + E2[1]*v,
                               V[2] + E1[2]*u + E2[2]*v)

        bounce_ray = gen_bounce_ray(normal, intersection_coords, d)

        reflection = get_intersection(bubble_tree, bounce_ray, max_reflection, reflection_depth+1)

        if reflection != None:
            pix_RGB = vec_tools.mixRGB(pix_RGB, reflection, 0.5)

    if reflection_depth > reflection_count:
        reflection_count = reflection_depth

    return pix_RGB


def main():
    # first define some constants
    # space is in X,Y,Z where X is the front, Y is sideways and Z is vertical
    viewer_origin = (0,0,0)
    resolution = (720*2,720*2)

    # create the image output buffer
    res_image = np.zeros((resolution[1], resolution[0], 3), dtype=np.uint8)

    triangles_vec = []
    triangle_vertices = read_stl("STLs/mesh.stl")
    triangles_vec = gen_triangle_vectors(triangle_vertices, (0, 0, 255))

    triangle_vertices = read_stl("STLs/refl_plane.stl")
    triangles_vec = triangles_vec + gen_triangle_vectors(triangle_vertices, (100, 100, 100))

    print("Number of triangles:", len(triangles_vec))

    bubble_tree = make_tree(triangles_vec)

    print("init done")
    startt = time.time()

    screen = make_screen(2330*2, resolution)

    for pixel_x in range(resolution[0]):
        for pixel_y in range(resolution[1]):
            ray = gen_ray(pixel_x, pixel_y, screen, viewer_origin)
            pix_RGB = get_intersection(bubble_tree, ray, max_reflection=4)

            if pix_RGB != None:
                res_image[pixel_y, pixel_x] = pix_RGB

        cv2.imshow("wow", res_image)
        cv2.waitKey(1)
        print(pixel_x, end="\r")

    print("done")
    endt = time.time()

    imloc = "outputs/{}.png".format(int(time.time()))
    cv2.imwrite(imloc, res_image)
    cv2.imshow("wow", res_image)
    cv2.waitKey(1)
    cv2.destroyAllWindows()

    return endt-startt


TTC = main()
print("Time to completion: ", TTC)
