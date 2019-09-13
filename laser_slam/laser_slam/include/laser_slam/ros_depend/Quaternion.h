/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef TF_QUATERNION_H_
#define TF_QUATERNION_H_


#include "Vector3.h"
#include "QuadWord.h"

#include "macros.h"

namespace tf
{

    class Quaternion : public QuadWord {
    public:
        Quaternion() {}


        //              template <typename tfScalar>
        //              explicit Quaternion(const tfScalar *v) : Tuple4<tfScalar>(v) {}
        Quaternion(const tfScalar& x, const tfScalar& y, const tfScalar& z, const tfScalar& w)
                : QuadWord(x, y, z, w)
        {}
        Quaternion(const Vector3& axis, const tfScalar& angle)
        {
            setRotation(axis, angle);
        }
        ROS_DEPRECATED Quaternion(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll)
        {
#ifndef TF_EULER_DEFAULT_ZYX
            setEuler(yaw, pitch, roll);
#else
            setRPY(roll, pitch, yaw);
#endif
        }
        void setRotation(const Vector3& axis, const tfScalar& angle)
        {
            tfScalar d = axis.length();
            tfAssert(d != tfScalar(0.0));
            tfScalar s = tfSin(angle * tfScalar(0.5)) / d;
            setValue(axis.x() * s, axis.y() * s, axis.z() * s,
                     tfCos(angle * tfScalar(0.5)));
        }
        void setEuler(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll)
        {
            tfScalar halfYaw = tfScalar(yaw) * tfScalar(0.5);
            tfScalar halfPitch = tfScalar(pitch) * tfScalar(0.5);
            tfScalar halfRoll = tfScalar(roll) * tfScalar(0.5);
            tfScalar cosYaw = tfCos(halfYaw);
            tfScalar sinYaw = tfSin(halfYaw);
            tfScalar cosPitch = tfCos(halfPitch);
            tfScalar sinPitch = tfSin(halfPitch);
            tfScalar cosRoll = tfCos(halfRoll);
            tfScalar sinRoll = tfSin(halfRoll);
            setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
                     cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
                     sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
                     cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
        }
        void setRPY(const tfScalar& roll, const tfScalar& pitch, const tfScalar& yaw)
        {
            tfScalar halfYaw = tfScalar(yaw) * tfScalar(0.5);
            tfScalar halfPitch = tfScalar(pitch) * tfScalar(0.5);
            tfScalar halfRoll = tfScalar(roll) * tfScalar(0.5);
            tfScalar cosYaw = tfCos(halfYaw);
            tfScalar sinYaw = tfSin(halfYaw);
            tfScalar cosPitch = tfCos(halfPitch);
            tfScalar sinPitch = tfSin(halfPitch);
            tfScalar cosRoll = tfCos(halfRoll);
            tfScalar sinRoll = tfSin(halfRoll);
            setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
                     cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
                     cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
                     cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
        }
        ROS_DEPRECATED void setEulerZYX(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll)
        {
            setRPY(roll, pitch, yaw);
        }
        TFSIMD_FORCE_INLINE     Quaternion& operator+=(const Quaternion& q)
        {
            m_floats[0] += q.x(); m_floats[1] += q.y(); m_floats[2] += q.z(); m_floats[3] += q.m_floats[3];
            return *this;
        }

        Quaternion& operator-=(const Quaternion& q)
        {
            m_floats[0] -= q.x(); m_floats[1] -= q.y(); m_floats[2] -= q.z(); m_floats[3] -= q.m_floats[3];
            return *this;
        }

        Quaternion& operator*=(const tfScalar& s)
        {
            m_floats[0] *= s; m_floats[1] *= s; m_floats[2] *= s; m_floats[3] *= s;
            return *this;
        }

        Quaternion& operator*=(const Quaternion& q)
        {
            setValue(m_floats[3] * q.x() + m_floats[0] * q.m_floats[3] + m_floats[1] * q.z() - m_floats[2] * q.y(),
                     m_floats[3] * q.y() + m_floats[1] * q.m_floats[3] + m_floats[2] * q.x() - m_floats[0] * q.z(),
                     m_floats[3] * q.z() + m_floats[2] * q.m_floats[3] + m_floats[0] * q.y() - m_floats[1] * q.x(),
                     m_floats[3] * q.m_floats[3] - m_floats[0] * q.x() - m_floats[1] * q.y() - m_floats[2] * q.z());
            return *this;
        }
        tfScalar dot(const Quaternion& q) const
        {
            return m_floats[0] * q.x() + m_floats[1] * q.y() + m_floats[2] * q.z() + m_floats[3] * q.m_floats[3];
        }

        tfScalar length2() const
        {
            return dot(*this);
        }

        tfScalar length() const
        {
            return tfSqrt(length2());
        }

        Quaternion& normalize()
        {
            return *this /= length();
        }

        TFSIMD_FORCE_INLINE Quaternion
        operator*(const tfScalar& s) const
        {
            return Quaternion(x() * s, y() * s, z() * s, m_floats[3] * s);
        }


        Quaternion operator/(const tfScalar& s) const
        {
            tfAssert(s != tfScalar(0.0));
            return *this * (tfScalar(1.0) / s);
        }

        Quaternion& operator/=(const tfScalar& s)
        {
            tfAssert(s != tfScalar(0.0));
            return *this *= tfScalar(1.0) / s;
        }

        Quaternion normalized() const
        {
            return *this / length();
        }
        tfScalar angle(const Quaternion& q) const
        {
            tfScalar s = tfSqrt(length2() * q.length2());
            tfAssert(s != tfScalar(0.0));
            return tfAcos(dot(q) / s);
        }
        tfScalar angleShortestPath(const Quaternion& q) const
        {
            tfScalar s = tfSqrt(length2() * q.length2());
            tfAssert(s != tfScalar(0.0));
            if (dot(q) < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
                return tfAcos(dot(-q) / s) * tfScalar(2.0);
            else
                return tfAcos(dot(q) / s) * tfScalar(2.0);
        }
        tfScalar getAngle() const
        {
            tfScalar s = tfScalar(2.) * tfAcos(m_floats[3]);
            return s;
        }

        tfScalar getAngleShortestPath() const
        {
            tfScalar s;
            if (m_floats[3] < 0)
                s = tfScalar(2.) * tfAcos(-m_floats[3]);
            else
                s = tfScalar(2.) * tfAcos(m_floats[3]);
            return s;
        }

        Vector3 getAxis() const
        {
            tfScalar s_squared = tfScalar(1.) - tfPow(m_floats[3], tfScalar(2.));
            if (s_squared < tfScalar(10.) * TFSIMD_EPSILON) //Check for divide by zero
                return Vector3(1.0, 0.0, 0.0);  // Arbitrary
            tfScalar s = tfSqrt(s_squared);
            return Vector3(m_floats[0] / s, m_floats[1] / s, m_floats[2] / s);
        }

        Quaternion inverse() const
        {
            return Quaternion(-m_floats[0], -m_floats[1], -m_floats[2], m_floats[3]);
        }

        TFSIMD_FORCE_INLINE Quaternion
        operator+(const Quaternion& q2) const
        {
            const Quaternion& q1 = *this;
            return Quaternion(q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z(), q1.m_floats[3] + q2.m_floats[3]);
        }

        TFSIMD_FORCE_INLINE Quaternion
        operator-(const Quaternion& q2) const
        {
            const Quaternion& q1 = *this;
            return Quaternion(q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z(), q1.m_floats[3] - q2.m_floats[3]);
        }

        TFSIMD_FORCE_INLINE Quaternion operator-() const
        {
            const Quaternion& q2 = *this;
            return Quaternion( - q2.x(), - q2.y(),  - q2.z(),  - q2.m_floats[3]);
        }
        TFSIMD_FORCE_INLINE Quaternion farthest( const Quaternion& qd) const
        {
            Quaternion diff,sum;
            diff = *this - qd;
            sum = *this + qd;
            if( diff.dot(diff) > sum.dot(sum) )
                return qd;
            return (-qd);
        }

        TFSIMD_FORCE_INLINE Quaternion nearest( const Quaternion& qd) const
        {
            Quaternion diff,sum;
            diff = *this - qd;
            sum = *this + qd;
            if( diff.dot(diff) < sum.dot(sum) )
                return qd;
            return (-qd);
        }


        Quaternion slerp(const Quaternion& q, const tfScalar& t) const
        {
            tfScalar theta = angleShortestPath(q) / tfScalar(2.0);
            if (theta != tfScalar(0.0))
            {
                tfScalar d = tfScalar(1.0) / tfSin(theta);
                tfScalar s0 = tfSin((tfScalar(1.0) - t) * theta);
                tfScalar s1 = tfSin(t * theta);
                if (dot(q) < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
                    return Quaternion((m_floats[0] * s0 + -q.x() * s1) * d,
                                      (m_floats[1] * s0 + -q.y() * s1) * d,
                                      (m_floats[2] * s0 + -q.z() * s1) * d,
                                      (m_floats[3] * s0 + -q.m_floats[3] * s1) * d);
                else
                    return Quaternion((m_floats[0] * s0 + q.x() * s1) * d,
                                      (m_floats[1] * s0 + q.y() * s1) * d,
                                      (m_floats[2] * s0 + q.z() * s1) * d,
                                      (m_floats[3] * s0 + q.m_floats[3] * s1) * d);

            }
            else
            {
                return *this;
            }
        }

        static const Quaternion&        getIdentity()
        {
            static const Quaternion identityQuat(tfScalar(0.),tfScalar(0.),tfScalar(0.),tfScalar(1.));
            return identityQuat;
        }

        TFSIMD_FORCE_INLINE const tfScalar& getW() const { return m_floats[3]; }


    };


    TFSIMD_FORCE_INLINE Quaternion
    operator-(const Quaternion& q)
    {
        return Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
    }



    TFSIMD_FORCE_INLINE Quaternion
    operator*(const Quaternion& q1, const Quaternion& q2) {
        return Quaternion(q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
                          q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
                          q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
                          q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z());
    }

    TFSIMD_FORCE_INLINE Quaternion
    operator*(const Quaternion& q, const Vector3& w)
    {
        return Quaternion( q.w() * w.x() + q.y() * w.z() - q.z() * w.y(),
                           q.w() * w.y() + q.z() * w.x() - q.x() * w.z(),
                           q.w() * w.z() + q.x() * w.y() - q.y() * w.x(),
                           -q.x() * w.x() - q.y() * w.y() - q.z() * w.z());
    }

    TFSIMD_FORCE_INLINE Quaternion
    operator*(const Vector3& w, const Quaternion& q)
    {
        return Quaternion( w.x() * q.w() + w.y() * q.z() - w.z() * q.y(),
                           w.y() * q.w() + w.z() * q.x() - w.x() * q.z(),
                           w.z() * q.w() + w.x() * q.y() - w.y() * q.x(),
                           -w.x() * q.x() - w.y() * q.y() - w.z() * q.z());
    }

    TFSIMD_FORCE_INLINE tfScalar
    dot(const Quaternion& q1, const Quaternion& q2)
    {
        return q1.dot(q2);
    }


    TFSIMD_FORCE_INLINE tfScalar
    length(const Quaternion& q)
    {
        return q.length();
    }

    TFSIMD_FORCE_INLINE tfScalar
    angle(const Quaternion& q1, const Quaternion& q2)
    {
        return q1.angle(q2);
    }

    TFSIMD_FORCE_INLINE tfScalar
    angleShortestPath(const Quaternion& q1, const Quaternion& q2)
    {
        return q1.angleShortestPath(q2);
    }

    TFSIMD_FORCE_INLINE Quaternion
    inverse(const Quaternion& q)
    {
        return q.inverse();
    }

    TFSIMD_FORCE_INLINE Quaternion
    slerp(const Quaternion& q1, const Quaternion& q2, const tfScalar& t)
    {
        return q1.slerp(q2, t);
    }

    TFSIMD_FORCE_INLINE Vector3
    quatRotate(const Quaternion& rotation, const Vector3& v)
    {
        Quaternion q = rotation * v;
        q *= rotation.inverse();
        return Vector3(q.getX(),q.getY(),q.getZ());
    }

    TFSIMD_FORCE_INLINE Quaternion
    shortestArcQuat(const Vector3& v0, const Vector3& v1) // Game Programming Gems 2.10. make sure v0,v1 are normalized
    {
        Vector3 c = v0.cross(v1);
        tfScalar  d = v0.dot(v1);

        if (d < -1.0 + TFSIMD_EPSILON)
        {
            Vector3 n,unused;
            tfPlaneSpace1(v0,n,unused);
            return Quaternion(n.x(),n.y(),n.z(),0.0f); // just pick any vector that is orthogonal to v0
        }

        tfScalar  s = tfSqrt((1.0f + d) * 2.0f);
        tfScalar rs = 1.0f / s;

        return Quaternion(c.getX()*rs,c.getY()*rs,c.getZ()*rs,s * 0.5f);
    }

    TFSIMD_FORCE_INLINE Quaternion
    shortestArcQuatNormalize2(Vector3& v0,Vector3& v1)
    {
        v0.normalize();
        v1.normalize();
        return shortestArcQuat(v0,v1);
    }

}
#endif




