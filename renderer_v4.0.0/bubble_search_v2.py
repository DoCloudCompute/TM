import vec_tools
from STL_reader import read_stl


def find_farthest_val(list_of_points, target):
    dist_lst = []
    for point in list_of_points:
        distance_squared = vec_tools.points_separation_sq(point, target)
        dist_lst.append(distance_squared)

    return max(dist_lst)


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
        tri_bubble = [centroid, radius_squared, [A, E1, E2, tri_vec_element]]
        tri_bubble = [centroid, radius_squared, ["a"]]
        triangles_vec.append(tri_bubble)

    return triangles_vec


def find_farthest_id(center, triangle_bubbles):
    biggest_dist = None
    for tri_id, tri in enumerate(triangle_bubbles):
        tri_centroid = tri[0]
        tri_radius = tri[1]
        dist = vec_tools.points_separation_sq(tri_centroid, center) + tri_radius

        if biggest_dist == None or dist > biggest_dist:
            biggest_dist = dist
            farthest_tri_id = tri_id

    return biggest_dist, triangle_bubbles[farthest_tri_id]


def make_bubble_from_tris(triangle_bubbles):
    # first get centroid by making a bubble
    global_centroid = (0, 0, 0)
    for tri in triangle_bubbles:
        global_centroid = vec_tools.add3(global_centroid, tri[0])
    global_centroid = vec_tools.div3_scalar(global_centroid, len(triangle_bubbles))

    global_radius, farthest_tri = find_farthest_id(global_centroid, triangle_bubbles)

    return global_centroid, global_radius, farthest_tri


def sort_by_dist(main_tri, triangle_bubbles):
    main_centroid = main_tri[0]
    dist_lst = {}
    for tri_id, tri in enumerate(triangle_bubbles):
        tri_centroid = tri[0]
        dist = vec_tools.points_separation_sq(tri_centroid, main_centroid)
        dist_lst[tri_id] = dist

    # sort by distance
    sorted_dist_lst_tup = sorted(dist_lst.items(), key=lambda x: x[1])

    sorted_lst = []
    for thing in sorted_dist_lst_tup:
        sorted_lst.append(thing[0])

    return sorted_lst


def make_tree(triangle_bubbles):
    """
    main centroid is going to be the average of all triangles centroids
    for next bubble level, take triangle furthest away and start grouping up 1/3 of the total triangles
    """

    centroid, radius, first_tri = make_bubble_from_tris(triangle_bubbles)

    # from there, get the first third of the triangles that are closest to the farthest triangle
    # get sorted list (by distance) of triangles
    group1_ids = sort_by_dist(first_tri, triangle_bubbles)
    first_slice = int(len(triangle_bubbles)/3)
    group1_ids = group1_ids[:first_slice]

    group1 = []
    for id in group1_ids:
        group1.append(triangle_bubbles[id])
    for id in sorted(group1_ids, reverse=True):
        del triangle_bubbles[id]

    if len(group1) <= 3:
        sub_bubble1 = group1
    else:
        sub_bubble1 = make_tree(group1)


    junk, second_tri = find_farthest_id(centroid, triangle_bubbles)
    group2_ids = sort_by_dist(second_tri, triangle_bubbles)
    second_slice = int(len(triangle_bubbles)/2)
    group2_ids = group2_ids[:second_slice]

    group2 = []
    for id in group2_ids:
        group2.append(triangle_bubbles[id])
    for id in sorted(group2_ids, reverse=True):
        del triangle_bubbles[id]

    if len(group2) <= 3:
        sub_bubble2 = group2
    else:
        sub_bubble2 = make_tree(group2)


    group3 = triangle_bubbles
    if len(group3) <= 3:
        sub_bubble3 = group3
    else:
        sub_bubble3 = make_tree(group3)

    return [centroid, radius, [sub_bubble1, sub_bubble2, sub_bubble3]]




triangles = read_stl("STLs/icosphere.stl")
tri_vecs = gen_triangle_vectors(triangles, (255, 255, 255))
res = make_tree(tri_vecs)
print(res)
