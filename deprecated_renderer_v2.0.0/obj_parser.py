def parse_textured_face(line):
    line_elems = line.split(" ")[1:]

    vertices = []
    texture_coords = []
    normals = []

    for idx_group in line_elems:
        idxs = idx_group.split("/")
        vertices.append(int(idxs[0])-1)
        texture_coords.append(int(idxs[1])-1)
        normals.append(int(idxs[2])-1)

    return vertices, texture_coords, normals

def parse_normaled_faces(line):
    line_elems = line.split(" ")[1:]

    vertices = []
    normals = []

    for idx_group in line_elems:
        idxs = idx_group.split("//")
        vertices.append(int(idxs[0])-1)
        normals.append(int(idxs[1])-1)

    return vertices, normals

def parse_vertices(line):
    vert_coords = line.split(" ")[1:]
    vert = (float(vert_coords[0]), float(vert_coords[1]), float(vert_coords[2]))

    return vert

def parse_vertex_normals(line):
    vert_normal_coords = line.split(" ")[1:]
    vert_normal = (float(vert_normal_coords[0]), float(vert_normal_coords[1]), float(vert_normal_coords[2]))

    return vert_normal

def parse_file(filepath):
    obj_f = open(filepath, "r")
    obj = obj_f.read()
    obj_f.close()

    obj_lines = obj.split("\n")

    vertices = []
    vertices_normals = []
    triangles_verts = []
    triangle_norms = []
    triangle_vertex_UV = []

    for line in obj_lines:
        if line.startswith("v "):
            vert = parse_vertices(line)
            vertices.append(vert)

        elif line.startswith("vn "):
            vert_normal = parse_vertex_normals(line)
            vertices_normals.append(vert_normal)

        elif line.startswith("f ") and "//" in line:
            face_dats, face_norm = parse_normaled_faces(line)

            triangles_verts.append(face_dats)
            triangle_norms.append(face_norm)

        elif line.startswith("f ") and "/" in line:
            face_dats, face_tex, face_norm = parse_textured_face(line)

            triangles_verts.append(face_dats)
            triangle_norms.append(face_norm)
            triangle_vertex_UV.append(face_tex)

    return vertices, triangles_verts
