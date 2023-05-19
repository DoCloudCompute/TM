import vec_tools
from STL_reader import read_stl
from bubble_search_v3 import make_tree
#from bubble_search_v3 import triangle_wrapper
import numpy as np
import cv2
import time
from math import acos, asin, atan, tan, degrees

"""
Globals
"""
hdri_img = cv2.imread("HDRIs/carpentry_shop_02_4k.png", 1)
reflection_count = 0
walk_call_count = 0
ray_count = 0


# first, define screen, generator for rays
def make_screen(screen_distance, resolution):
    corner = (screen_distance, resolution[0]/2, resolution[1]/2) # divide by 2 so that the plane is centered
    edge1 = (0, -1, 0) # on Y axis
    edge2 = (0, 0, -1) # on Z axis

    return corner, edge1, edge2


def gen_ray(pixel_x, pixel_y, screen, viewer_origin):
    global ray_count
    ray_count += 1

    # point of intersection
    S = (screen[0][0] + screen[1][0]*pixel_x + screen[2][0]*pixel_y,
         screen[0][1] + screen[1][1]*pixel_x + screen[2][1]*pixel_y,
         screen[0][2] + screen[1][2]*pixel_x + screen[2][2]*pixel_y)

    d = vec_tools.sub3(S, viewer_origin) # directing vector
    d = vec_tools.unit3(d)
    return viewer_origin, d


def gen_bounce_ray(normal, origin, d):
    global ray_count
    ray_count += 1

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
    global walk_call_count
    walk_call_count += 1
    """
    first check the number of elements in list, if theres 3 of em, its a bubble
    otherwise, its a triangle so return just that

    get 2 first list elems, see if intersect or no
    if yes, call again for each sub element
    """

    if len(bubble_tree[2]) == 5:
        return does_intersect(bubble_tree[2], ray)

    O, d = ray
    centroid, radius_squared = bubble_tree[0:2]

    # CRM page 64
    OC = vec_tools.sub3(centroid, O)


    intersect = False
    if vec_tools.norm3_sq(OC) <= radius_squared:    # case origin in bubble
        intersect = True

    elif vec_tools.dot3(d, OC) < 0:                 # case bubble behind origin
        return

    else:                                           # case bubble intersects ray ahead
        OC_cross_d = vec_tools.cross(OC, d)
        dist_sq = vec_tools.norm3_sq(OC_cross_d)
        if dist_sq <= radius_squared:
            intersect = True

    if intersect == True:
        groups = bubble_tree[2]

        '''
        get bubble group stats and extract dist, put it in dict and sort by dist
        '''
        dist_by_id = {}
        for idx, group in enumerate(groups):
            centroid, radius = group[0], group[1]
            OC = vec_tools.sub3(centroid, O)
            dist_sq = vec_tools.norm3_sq(OC) + radius

            dist_by_id[idx] = dist_sq
        dist_sorted_bubbles = dict(sorted(dist_by_id.items(), key=lambda item: item[1]))

        for bubble_idx in dist_sorted_bubbles.keys():
            res = walk_tree(groups[bubble_idx], ray)
            if res != None:
                return res

    return

def does_intersect(triangle, ray):
    O, d = ray # unpack origin and directing vector

    V, E1, E2, normal, color = triangle

    # backface culling
    if vec_tools.dot3(d, normal) >= 0: return

    VO = vec_tools.sub3(O, V)

    cross1 = normal
    neg_d = vec_tools.negative(d)
    pre_invdet = vec_tools.dot3(d, cross1)

    if abs(pre_invdet) < 1e-8: return

    invdet = -1/pre_invdet

    t = vec_tools.dot3(VO, cross1) * invdet
    if t < 1e-6: return

    cross2 = vec_tools.cross(VO, neg_d)

    neg_E2 = vec_tools.negative(E2)
    u = vec_tools.dot3(neg_E2, cross2) * invdet
    if u > 1 or u < 0: return

    v = vec_tools.dot3(E1, cross2) * invdet

    if u+v <= 1 and u >= 0 and v >= 0:
        return u, v, triangle, color
    
    return


def get_intersection(bubble_tree, ray, max_reflection, reflection_depth = 0):
    global reflection_count

    intersect = walk_tree(bubble_tree, ray)

    if not intersect: return get_HDRI(ray)
    else: intersected_u, intersected_v, intersected_tri, color = intersect

    pix_RGB = color # set color

    _, d = ray # unpack origin and directing vector

    # if the desired trace is not yet complete, keep reflecting
    if reflection_depth < max_reflection and intersected_tri != None:
        # origin of bounce ray
        V, E1, E2, normal, _ = intersected_tri
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
    resolution = (720,720)

    fov_angle = 30

    screen_distance = -1 * (resolution[0]/2) / (tan(fov_angle/2))

    screen_distance = 2330

    # create the image output buffer
    res_image = np.zeros((resolution[1], resolution[0], 3), dtype=np.uint8)

    triangle_vertices = read_stl("STLs/mesh.stl", texture = (0, 0, 255))
    triangle_vertices += read_stl("STLs/refl_plane.stl", texture = (100, 100, 100))

    print("Number of triangles:", len(triangle_vertices))

    bubble_tree = make_tree(triangle_vertices)

    print("init done")
    startt = time.time()

    screen = make_screen(screen_distance, resolution)

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

    imloc = "../outputs/{}.png".format(int(time.time()))
    cv2.imwrite(imloc, res_image)
    cv2.imshow("wow", res_image)
    cv2.waitKey(1)
    cv2.destroyAllWindows()

    return endt-startt


TTC = main()
print("Time to completion: ", TTC)

print('Walk tree call count: {}, call per ray: {}, ray count: {}'.format(walk_call_count, walk_call_count/ray_count, ray_count))
