import vec_tools
from STL_reader import read_stl


def find_farthest_val(list_of_points, target):
    dist_lst = []
    for point in list_of_points:
        distance_squared = vec_tools.points_separation_sq(point, target)
        dist_lst.append(distance_squared)

    return max(dist_lst)


def find_closest_id(list_of_bubbles, target):
    smallest_dist = None

    for bubble in list_of_bubbles:
        point = bubble[0]
        distance_squared = vec_tools.points_separation_sq(point, target)

        if distance_squared != 0:
            if smallest_dist == None:
                smallest_dist = distance_squared




def gen_triangle_vectors(triangle_vertices, color):
    # color are BGR (Blue, Green, Red)
    # transform triangles to triangles_vec (E1 = B - A | E2 = C - A)
    triangles_vec = []
    for tri in triangle_vertices:
        A = tri[0]
        B = tri[1]
        C = tri[2]

        centroid = vec_tools.centroid3(A, B, C)

        radius_squared = find_farthest_val([A, B, C], centroid)

        E1 = vec_tools.sub3(B, A)
        E2 = vec_tools.sub3(C, A)

        normal = vec_tools.cross(E1, E2)

        tri_vec_element = [A, E1, E2, normal, color]
        tri_bubble = [centroid, radius_squared, [A, E1, E2, tri_vec_element]
        triangles_vec.append(tri_bubble)

    return triangles_vec


def group_triangles_lv1(triangles_vec):
    """
    The idea with this function is that it make a bubble with 3 triangles innit olright
    """

    triangle_groups = []
    for bubble_id, bubble in enumerate(triangles_vec):
        centroid_1 = bubble[0]
        radius_squared_1 = bubble[1]





        for rpt in range(2):
            closest_dist_squared = 0
            for i, test_triangle in enumerate(triangles_vec):
                test_centroid = test_triangle[4]

                dist_vec = vec_tools.sub3(test_centroid, centroid)
                dist_squared = vec_tools.norm3_sq(dist_vec)
                if dist_squared < closest_dist_squared:
                    closest_dist_squared = dist_squared



triangles = read_stl("STLs/refl_plane.stl")


"""
farthest_dist_squared = 0
for vertex in face:
    centroid_vertex_vec = vec_tools.sub3(vertex, centroid)
    dist_squared = vec_tools.norm3_sq(centroid_vertex_vec)
    if dist_squared > farthest_dist_squared:
        farthest_dist_squared = dist_squared
"""
