import vec_tools


"""
kinda started from this:
https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.cuemath.com%2Fcentroid-formula%2F&psig=AOvVaw1axA-8I0hsGYfFaWFA6kGI&ust=1628840411203000&source=images&cd=vfe&ved=0CAoQjRxqFwoTCMCAmYK12PICFQAAAAAdAAAAABAD
"""

'''
triangle spec:
[
    vertex1: vec3,
    vertex2: vec3,
    vertex3: vec3,
    normal:  vec3
]
'''

def gen_triangle_vectors(triangle):
    # color are BGR (Blue, Green, Red)
    # transform triangles to triangles_vec (E1 = B - A | E2 = C - A)
    A, B, C, _, color = triangle

    E1 = vec_tools.sub3(B, A)
    E2 = vec_tools.sub3(C, A)
    normal = vec_tools.cross(E1, E2)

    triangle_vec = [A, E1, E2, normal, color]

    return triangle_vec


def triangle_wrapper(triangles):
    wrapped_triangles = []

    for tri in triangles:
        centroid = vec_tools.centroid3(tri[0], tri[1], tri[2])
        radius = max([vec_tools.points_separation_sq(centroid, vertex) for vertex in tri[:3]])

        wrapped_triangles.append([centroid, radius, gen_triangle_vectors(tri)])

    return wrapped_triangles


def bubble_wrapper(*bubbles):
    if len(bubbles) == 1:
        centroid, radius, _ = bubbles[0]

    elif len(bubbles) == 2:
        centroid = vec_tools.centroid2(bubbles[0][0], bubbles[1][0])
        radius = max([vec_tools.points_separation(centroid, bubble[0]) + bubble[1]**0.5 for bubble in bubbles]) **2 # distance from centroid of parent bubble to centroid of child bubble + radius of child bubble

    elif len(bubbles) == 3:
        centroid = vec_tools.centroid3(bubbles[0][0], bubbles[1][0], bubbles[2][0])
        radius = max([vec_tools.points_separation(centroid, bubble[0]) + bubble[1]**0.5 for bubble in bubbles]) **2 # distance from centroid of parent bubble to centroid of child bubble + radius of child bubble

    return [centroid, radius, bubbles]


def bubble_builder(child_bubbles):
    print(len(child_bubbles))

    if len(child_bubbles) == 1: return child_bubbles[0]
    parent_bubble = []
    done_idx = []

    for child_idx, child_bubble in enumerate(child_bubbles):
        if child_idx in done_idx: continue
        done_idx.append(child_idx)

        dist_by_id = {test_idx: vec_tools.points_separation_sq(child_bubble[0], test_bubble[0]) for test_idx, test_bubble in enumerate(child_bubbles) if test_idx not in done_idx}
        closest2_ids = list(dict(sorted(dist_by_id.items(), key=lambda item: item[1])).keys())[:2]
        

        done_idx += closest2_ids

        if len(closest2_ids) == 0: parent_bubble.append(child_bubble)
        elif len(closest2_ids) == 1: parent_bubble.append(bubble_wrapper(child_bubble, child_bubbles[closest2_ids[0]]))
        elif len(closest2_ids) == 2: parent_bubble.append(bubble_wrapper(child_bubble, child_bubbles[closest2_ids[0]], child_bubbles[closest2_ids[1]]))

    return bubble_builder(parent_bubble)


def make_tree(triangles):
    lv1_bubble = triangle_wrapper(triangles)
    return bubble_builder(lv1_bubble)
