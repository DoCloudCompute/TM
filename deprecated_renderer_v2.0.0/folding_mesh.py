import obj_parser
import numpy as np
import cv2
import graphics

"""
Step 0: define some operations
"""
def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])

def det3(a, b, c):
    return a[0]*b[1]*c[2] + b[0]*c[1]*a[2] + c[0]*a[1]*b[2] - c[0]*b[1]*a[2] - b[0]*a[1]*c[2] - a[0]*c[1]*b[2]

def det2(a, b):
    return a[0]*b[1] - b[0]*a[1]

def dot3(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def dot2(a, b):
    return a[0]*b[0] + a[1]*b[1]


"""
Step 1: init camera and view
"""
pixels_x = int(720/1)
pixels_y = int(720/1)
res_image = np.zeros((pixels_y, pixels_x, 3), dtype=np.uint8)

ray_origin = (-3, 0, 0)
screen_corner = (-1, -1, -1)
screen_edges = [(0, 2, 0), (0, 0, 2)]

screen_origin_2d = (0.5 * pixels_x,
                    0.5 * pixels_y)

screen_edges_2d = [(-pixels_x, 0), (0, -pixels_y)]


"""
Step 2: get the cartesian coefficients of the screen plane
"""
screen_normal = cross(screen_edges[0], screen_edges[1])
d_offset = dot3(screen_normal, screen_corner)


"""
Step 3: load vertices and faces
"""
verts, faces = obj_parser.parse_file("../obj_to_vtr/sphere.obj")


"""
Step 4: find closest vertex and build vertex distance list
"""
distance_lst = [0]*len(verts)
smallest_distance = None
closest_vtx = None

for v_id, vertex in enumerate(verts):
    distance = abs(dot3(vertex, screen_normal) + d_offset)
    distance_lst[v_id] = distance

    if not smallest_distance or distance < smallest_distance:
        smallest_distance = distance
        closest_vtx = v_id

"""
Step 5: do 2D projection
"""
twoD_verts = [0]*len(verts)

d = (ray_origin[0] - screen_corner[0],
     ray_origin[1] - screen_corner[1],
     ray_origin[2] - screen_corner[2])

for v_id, vertex in enumerate(verts):
    ray = (vertex[0] - ray_origin[0],
           vertex[1] - ray_origin[1],
           vertex[2] - ray_origin[2])

    cross_Du = cross(screen_edges[1], ray)
    main_det = dot3(cross_Du, screen_edges[0])

    if round(main_det, 6) != 0:
        inv_det = 1/main_det

        u_numerator_det = dot3(d, cross_Du)
        u = u_numerator_det * inv_det

        v_numerator_det = det3(screen_edges[0], d, ray)
        v = v_numerator_det * inv_det

        if u >= 0 and u <= 1 and v >= 0 and v <= 1:
            UV_coords = (int(screen_edges_2d[0][0]*u + screen_edges_2d[1][0]*v + pixels_x),
                         int(screen_edges_2d[0][1]*u + screen_edges_2d[1][1]*v + pixels_y))
            twoD_verts[v_id] = UV_coords

"""
Step 6: find matching triangle
"""
center_facet = None
for f_id, facet in enumerate(faces):
    if closest_vtx in facet:
        center_facet = f_id
        center_verts = facet
        break


"""
Step 7: Find closest connected face
Two vertices must be common
"""
frontier_vertices = center_verts
done_faces = []
center_coords = (twoD_verts[closest_vtx][0],
                 twoD_verts[closest_vtx][1])

while True:
    smallest_d = None
    next_vtx = None

    for f_id, facet in enumerate(faces):
        vtx_id_intersect = set(facet).intersection(frontier_vertices)

        if f_id not in done_faces:
            print(vtx_id_intersect, facet)
            if len(vtx_id_intersect) == 3:
                intersect = list(vtx_id_intersect)[0:2]
                next_vtx = list(vtx_id_intersect)[2]
                done_face_id = f_id
                break

            elif len(vtx_id_intersect) == 2:
                lone_vtx = list(set(facet)-vtx_id_intersect)[0]
                dist = distance_lst[lone_vtx]

                if not smallest_d or dist < smallest_d:
                    intersect_points = (twoD_verts[list(vtx_id_intersect)[0]],
                                        twoD_verts[list(vtx_id_intersect)[1]])

                    vertex1_coords = (intersect_points[0][0], intersect_points[0][1])
                    vertex2_coords = (intersect_points[1][0], intersect_points[1][1])

                    vertex1_vec = (intersect_points[0][0] - center_coords[0],
                                   intersect_points[0][1] - center_coords[1])

                    vertex2_vec = (intersect_points[1][0] - center_coords[0],
                                   intersect_points[1][1] - center_coords[1])

                    vertexSum_vec = (vertex1_vec[0]+vertex2_vec[0],
                                     vertex1_vec[1]+vertex2_vec[1])

                    lone_vec = (twoD_verts[lone_vtx][0] - center_coords[0],
                                twoD_verts[lone_vtx][1] - center_coords[1])

                    vec_dir = dot2(lone_vec, vertexSum_vec)
                    if vec_dir:
                        smallest_d = dist
                        next_vtx = lone_vtx
                        intersect = list(vtx_id_intersect)
                        done_face_id = f_id

    done_faces.append(done_face_id)
    if next_vtx: frontier_vertices.append(next_vtx)
    else: break

    res_image = graphics.showimg(faces, frontier_vertices, twoD_verts, res_image, next_vtx, intersect)
    #print(done_faces)
    print("")
