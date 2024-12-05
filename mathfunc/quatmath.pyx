# distutils: language=c++

from libcpp cimport cmath

cdef struct Quat:
    float w, x, y, z


cdef q2t(Quat q):
    return q.w, q.x, q.y, q.z


cdef t2q(t: tuple):
    return Quat(t[0], t[1], t[2], t[3])


cdef v2q(v: tuple):
    return Quat(0, v[0], v[1], v[2])


cdef _normalize(Quat q, float tolerance=0.00001):
    mag2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z
    if cmath.fabs(mag2 - 1.0) > tolerance:
        mag = cmath.sqrt(mag2)
        q.w /= mag
        q.x /= mag
        q.y /= mag
        q.z /= mag
    return q


cdef _mult(Quat q1, Quat q2):
    w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z
    z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x
    return Quat(w, x, y, z)


cdef _conjugate(Quat q):
    return Quat(q.w, -q.x, -q.y, -q.z)


cdef _getchildlocalrot(Quat wp, Quat wc):
    return _mult(_conjugate(wp), wc)


cdef _axisangle_to_q(Quat q, float theta):
    q = _normalize(q)
    theta /= 2
    w = cmath.cos(theta)
    x = q.x * cmath.sin(theta)
    y = q.y * cmath.sin(theta)
    z = q.z * cmath.sin(theta)
    return Quat(w, x, y, z)



def normalize(t: tuple, tolerance=0.00001):
    return q2t(_normalize(t2q(t), tolerance))


def mult(t1: tuple, t2: tuple):
    return q2t(_mult(t2q(t1), t2q(t2)))


def conjugate(t: tuple):
    return q2t(_conjugate(t2q(t)))


def getchildlocalrot(parent_w: tuple, child_w: tuple):
    return q2t(_getchildlocalrot(t2q(parent_w), t2q(child_w)))


def axisangle_to_q(v: tuple, theta: float):
    return q2t(_axisangle_to_q(v2q(v), theta))

def dot(t1: tuple, t2: tuple):
    return t1[0]*t2[0] + t1[1]*t2[1] + t1[2]*t2[2] + t1[3]*t2[3]