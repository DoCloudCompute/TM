import vec_tools


def find_farthest_val(list_of_points, target):
    dist_lst = []
    for point in list_of_points:
        distance = vec_tools.points_separation(point, target)
        dist_lst.append(distance)

    return max(dist_lst)


def find_farthest_id(center, triangle_bubbles):
    biggest_dist = None
    for tri_id, tri in enumerate(triangle_bubbles):
        tri_centroid = tri[0]
        tri_radius = tri[1]
        dist = (vec_tools.points_separation(tri_centroid, center) + tri_radius)**2

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
        sub_bubble1 = []
        for tri in group1:
            sub_bubble1.append(tri[2])
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
        sub_bubble2 = []
        for tri in group2:
            sub_bubble2.append(tri[2])
    else:
        sub_bubble2 = make_tree(group2)


    group3 = triangle_bubbles
    if len(group3) <= 3:
        sub_bubble3 = []
        for tri in group3:
            sub_bubble3.append(tri[2])
    else:
        sub_bubble3 = make_tree(group3)

    return [centroid, radius, [sub_bubble1, sub_bubble2, sub_bubble3]]
