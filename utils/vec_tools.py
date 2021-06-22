def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])

def negative(a):
    return (- a[0],
            - a[1],
            - a[2])

def det3(a, b, c):
    return a[0]*b[1]*c[2] + b[0]*c[1]*a[2] + c[0]*a[1]*b[2] - c[0]*b[1]*a[2] - b[0]*a[1]*c[2] - a[0]*c[1]*b[2]

def det2(a, b):
    return a[0]*b[1] - b[0]*a[1]

def dot3(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def dot3x3(a, b, c):
    return a[0]*b[0]*c[0] + a[1]*b[1]*c[1] + a[2]*b[2]*c[2]

def dot2(a, b):
    return a[0]*b[0] + a[1]*b[1]

def vec3(a, b):
    return (a[0] - b[0],
            a[1] - b[1],
            a[2] - b[2])

def vec2(a, b):
    return (a[0] - b[0],
            a[1] - b[1])

def vec3_norm(a):
    return (a[0]**2 + a[1]**2 + a[2]**2)**0.5

def unit_vec3(a):
    return (a[0]/vec3_norm(a),
            a[1]/vec3_norm(a),
            a[2]/vec3_norm(a))
