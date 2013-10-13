#!/usr/bin/env python


# derived from: https://bitbucket.org/sirex/quaternion

import math

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion as ROSQuaternion

def dbl2str(n):
    d = abs(int(n));
    f = int((n - d) * 1000000);
    s = '%d.%d' % (d, f) if f > 0 else '%d' % d
    ss = '+' if n >= 0 else '-'
    return ss, s


class Quaternion(object):
    def __init__(q, w=0, x=0, y=0, z=0):

        if hasattr(w,'x') and hasattr(w,'y') and hasattr(w,'z'):
            # no it is possible to create a Quaternion from another object that has the neccessary attributes (i.e. ROS geometry_msgs/Quaternion)
            q.x = w.x
            q.y = w.y
            q.z = w.z
            if hasattr(w,'w'):
                q.w = w.w
            else:
                q.w = 0
        else:
            q.w = w
            q.x = x
            q.y = y
            q.z = z

    def __add__(q1, q2):
        return Quaternion(
            q1.w + q2.w,
            q1.x + q2.x,
            q1.y + q2.y,
            q1.z + q2.z,
        )

    def __sub__(q1, q2):
        return Quaternion(
            q1.w - q2.w,
            q1.x - q2.x,
            q1.y - q2.y,
            q1.z - q2.z,
        )

    def __mul__(q1, q2):
        return Quaternion(
            q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
            q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
            q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
            q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w,
        )

    def __div__(q1, q2):
        s = float(q2.w*q2.w + q2.x*q2.x + q2.y*q2.y + q2.z*q2.z)
        return Quaternion(
            (  q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z) / s,
            (- q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y) / s,
            (- q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x) / s,
            (- q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w) / s
        )

    def __abs__(q):
        return math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);

    def __neg__(q):
        return Quaternion(-q.w, -q.x, -q.y, -q.z)

    def __invert__(q):
        """Conjugate of Quaternion.

        >>> q = Quaternion((2, 2, 2, 2))
        >>> print(q)
        (2 + 2i + 2j + 2k)
        >>> print(~q)
        (2 - 2i - 2j - 2k)
        >>> print(~~q)
        (2 + 2i + 2j + 2k)

        """
        return Quaternion(q.w, -q.x, -q.y, -q.z)

    def as_tuple(self):
        return (self.w, self.x, self.y, self.z)


    def __str__(q):
        args = list(
            dbl2str(q.w) +
            dbl2str(q.x) +
            dbl2str(q.y) +
            dbl2str(q.z)
        )
        if args[0] == '+':
            args[0] = ''
        return '(%s%s %s %si %s %sj %s %sk)' % tuple(args)

    def normalize(q):
        """Convert Quaternion to Unit Quaternion.

        Unit Quaternion is Quaternion who's length is equal to 1.

        >>> q = Quaternion((1, 3, 3, 3))
        >>> q.normalize()
        >>> print(q) # doctest: +ELLIPSIS
        (0.1889822... + 0.5669467...i + 0.5669467...j + 0.5669467...k)

        """
        norm = abs(q)
        q.w = q.w / norm,
        q.x = q.x / norm,
        q.y = q.y / norm,
        q.z = q.z / norm,

    @staticmethod
    def identity():
        return Quaternion(1,0,0,0)

    @staticmethod
    def from_axis(rad, axis):
        sin_rad = math.sin(rad/2.0)
        return Quaternion(w=math.cos(rad/2.0),
                          x=sin_rad * axis.x,
                          y=sin_rad * axis.y,
                          z=sin_rad * axis.z)


    def as_vector3(self):
        return Vector3(self.x, self.y, self.z)


    def as_rosquaternion(self):
        return ROSQuaternion(w=self.w,x=self.x,y=self.y,z=self.z)


    def rotate_vector3(self, vector3):
        """Returns the given vector rotated by the this quaternion"""
        applied = self * Quaternion(vector3) * ~self
        return Vector3(x=applied.x,y=applied.y,z=applied.z)

    def inv_rotate_vector3(self, vector3):
        """Returns the given vector rotated by the inverse of this quaternion"""
        applied = ~self * Quaternion(vector3) * self
        return applied.as_vector3()