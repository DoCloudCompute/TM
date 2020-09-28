import numpy as np
import cv2

def showimg(faces, frontier_vertices, twoD_verts, res_image, lone_vtx, intersect):
    """
    Display image
    """
    tri_vtx = []
    tri_vtx.append(twoD_verts[intersect[0]])
    tri_vtx.append(twoD_verts[intersect[1]])
    tri_vtx.append(twoD_verts[lone_vtx])
    tri_vtx = np.array(tri_vtx)

    cv2.drawContours(res_image, [tri_vtx], 0, (lone_vtx,255-lone_vtx,255), -1)

    for pid, points in enumerate(twoD_verts):
        #points = twoD_verts[pts]
        if points == twoD_verts[lone_vtx]:
            cv2.circle(res_image, points, 4, (0,0,255))
        elif points == twoD_verts[intersect[0]] or points == twoD_verts[intersect[1]]:
            cv2.circle(res_image, points, 4, (0,255,0))
        else:
            cv2.circle(res_image, points, 4, (255,255,255))

    cv2.imshow("image", res_image)
    cv2.waitKey(0)
    return res_image
