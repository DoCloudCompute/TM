import struct

def read_stl(fname, texture = (0,0,0)):
    triangles = []

    stl = open(fname, "rb")
    stl_header = stl.read(80)
    stl_n_buf = stl.read(4)
    stl_n_tri = int.from_bytes(stl_n_buf, byteorder="little")

    for face in range(0, stl_n_tri):
        stl_vals = []
        for i in range(0, 12):
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

        tri_verts = [A, B, C, normal, texture]
        triangles.append(tri_verts)
        stl.read(2)
    return triangles
