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



#ifndef tfTransform_H
#define tfTransform_H


#include "Matrix3x3.h"

namespace tf
{

#define TransformData TransformDoubleData


    class Transform {

        Matrix3x3 m_basis;
        Vector3   m_origin;

    public:

        Transform() {}
        explicit TFSIMD_FORCE_INLINE Transform(const Quaternion& q,
                                               const Vector3& c = Vector3(tfScalar(0), tfScalar(0), tfScalar(0)))
                : m_basis(q),
                  m_origin(c)
        {}

        explicit TFSIMD_FORCE_INLINE Transform(const Matrix3x3& b,
                                               const Vector3& c = Vector3(tfScalar(0), tfScalar(0), tfScalar(0)))
                : m_basis(b),
                  m_origin(c)
        {}
        TFSIMD_FORCE_INLINE Transform (const Transform& other)
                : m_basis(other.m_basis),
                  m_origin(other.m_origin)
        {
        }
        TFSIMD_FORCE_INLINE Transform& operator=(const Transform& other)
        {
            m_basis = other.m_basis;
            m_origin = other.m_origin;
            return *this;
        }

        TFSIMD_FORCE_INLINE void mult(const Transform& t1, const Transform& t2) {
            m_basis = t1.m_basis * t2.m_basis;
            m_origin = t1(t2.m_origin);
        }

        /*              void multInverseLeft(const Transform& t1, const Transform& t2) {
                                Vector3 v = t2.m_origin - t1.m_origin;
                                m_basis = tfMultTransposeLeft(t1.m_basis, t2.m_basis);
                                m_origin = v * t1.m_basis;
                        }
                        */

        TFSIMD_FORCE_INLINE Vector3 operator()(const Vector3& x) const
        {
            return Vector3(m_basis[0].dot(x) + m_origin.x(),
                           m_basis[1].dot(x) + m_origin.y(),
                           m_basis[2].dot(x) + m_origin.z());
        }

        TFSIMD_FORCE_INLINE Vector3 operator*(const Vector3& x) const
        {
            return (*this)(x);
        }

        TFSIMD_FORCE_INLINE Quaternion operator*(const Quaternion& q) const
        {
            return getRotation() * q;
        }

        TFSIMD_FORCE_INLINE Matrix3x3&       getBasis()          { return m_basis; }
        TFSIMD_FORCE_INLINE const Matrix3x3& getBasis()    const { return m_basis; }

        TFSIMD_FORCE_INLINE Vector3&         getOrigin()         { return m_origin; }
        TFSIMD_FORCE_INLINE const Vector3&   getOrigin()   const { return m_origin; }

        Quaternion getRotation() const {
            Quaternion q;
            m_basis.getRotation(q);
            return q;
        }


        void setFromOpenGLMatrix(const tfScalar *m)
        {
            m_basis.setFromOpenGLSubMatrix(m);
            m_origin.setValue(m[12],m[13],m[14]);
        }

        void getOpenGLMatrix(tfScalar *m) const
        {
            m_basis.getOpenGLSubMatrix(m);
            m[12] = m_origin.x();
            m[13] = m_origin.y();
            m[14] = m_origin.z();
            m[15] = tfScalar(1.0);
        }

        TFSIMD_FORCE_INLINE void setOrigin(const Vector3& origin)
        {
            m_origin = origin;
        }

        TFSIMD_FORCE_INLINE Vector3 invXform(const Vector3& inVec) const;


        TFSIMD_FORCE_INLINE void setBasis(const Matrix3x3& basis)
        {
            m_basis = basis;
        }

        TFSIMD_FORCE_INLINE void setRotation(const Quaternion& q)
        {
            m_basis.setRotation(q);
        }


        void setIdentity()
        {
            m_basis.setIdentity();
            m_origin.setValue(tfScalar(0.0), tfScalar(0.0), tfScalar(0.0));
        }

        Transform& operator*=(const Transform& t)
        {
            m_origin += m_basis * t.m_origin;
            m_basis *= t.m_basis;
            return *this;
        }

        Transform inverse() const
        {
            Matrix3x3 inv = m_basis.transpose();
            return Transform(inv, inv * -m_origin);
        }

        Transform inverseTimes(const Transform& t) const;

        Transform operator*(const Transform& t) const;

        static const Transform& getIdentity()
        {
            static const Transform identityTransform(Matrix3x3::getIdentity());
            return identityTransform;
        }

        void    serialize(struct        TransformData& dataOut) const;

        void    serializeFloat(struct   TransformFloatData& dataOut) const;

        void    deSerialize(const struct        TransformData& dataIn);

        void    deSerializeDouble(const struct  TransformDoubleData& dataIn);

        void    deSerializeFloat(const struct   TransformFloatData& dataIn);

    };


    TFSIMD_FORCE_INLINE Vector3
    Transform::invXform(const Vector3& inVec) const
    {
        Vector3 v = inVec - m_origin;
        return (m_basis.transpose() * v);
    }

    TFSIMD_FORCE_INLINE Transform
    Transform::inverseTimes(const Transform& t) const
    {
        Vector3 v = t.getOrigin() - m_origin;
        return Transform(m_basis.transposeTimes(t.m_basis),
                         v * m_basis);
    }

    TFSIMD_FORCE_INLINE Transform
    Transform::operator*(const Transform& t) const
    {
        return Transform(m_basis * t.m_basis,
                         (*this)(t.m_origin));
    }

    TFSIMD_FORCE_INLINE bool operator==(const Transform& t1, const Transform& t2)
    {
        return ( t1.getBasis()  == t2.getBasis() &&
                 t1.getOrigin() == t2.getOrigin() );
    }


    struct  TransformFloatData
    {
        Matrix3x3FloatData      m_basis;
        Vector3FloatData        m_origin;
    };

    struct  TransformDoubleData
    {
        Matrix3x3DoubleData     m_basis;
        Vector3DoubleData       m_origin;
    };



    TFSIMD_FORCE_INLINE     void    Transform::serialize(TransformData& dataOut) const
    {
        m_basis.serialize(dataOut.m_basis);
        m_origin.serialize(dataOut.m_origin);
    }

    TFSIMD_FORCE_INLINE     void    Transform::serializeFloat(TransformFloatData& dataOut) const
    {
        m_basis.serializeFloat(dataOut.m_basis);
        m_origin.serializeFloat(dataOut.m_origin);
    }


    TFSIMD_FORCE_INLINE     void    Transform::deSerialize(const TransformData& dataIn)
    {
        m_basis.deSerialize(dataIn.m_basis);
        m_origin.deSerialize(dataIn.m_origin);
    }

    TFSIMD_FORCE_INLINE     void    Transform::deSerializeFloat(const TransformFloatData& dataIn)
    {
        m_basis.deSerializeFloat(dataIn.m_basis);
        m_origin.deSerializeFloat(dataIn.m_origin);
    }

    TFSIMD_FORCE_INLINE     void    Transform::deSerializeDouble(const TransformDoubleData& dataIn)
    {
        m_basis.deSerializeDouble(dataIn.m_basis);
        m_origin.deSerializeDouble(dataIn.m_origin);

    }

}

#endif





