// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Copyright 2010 Michael Smith, all rights reserved.

// Derived closely from:
/****************************************
 * 3D Vector Classes
 * By Bill Perone (billperone@yahoo.com)
 * Original: 9-16-2002
 * Revised: 19-11-2003
 *          11-12-2003
 *          18-12-2003
 *          06-06-2004
 *
 * � 2003, This code is provided "as is" and you can use it freely as long as
 * credit is given to Bill Perone in the application it is used in
 *
 * Notes:
 * if a*b = 0 then a & b are orthogonal
 * a%b = -b%a
 * a*(b%c) = (a%b)*c
 * a%b = a(cast to matrix)*b
 * (a%b).length() = area of parallelogram formed by a & b
 * (a%b).length() = a.length()*b.length() * sin(angle between a & b)
 * (a%b).length() = 0 if angle between a & b = 0 or a.length() = 0 or b.length() = 0
 * a * (b%c) = volume of parallelpiped formed by a, b, c
 * vector triple product: a%(b%c) = b*(a*c) - c*(a*b)
 * scalar triple product: a*(b%c) = c*(a%b) = b*(c%a)
 * vector quadruple product: (a%b)*(c%d) = (a*c)*(b*d) - (a*d)*(b*c)
 * if a is unit vector along b then a%b = -b%a = -b(cast to matrix)*a = 0
 * vectors a1...an are linearly dependant if there exists a vector of scalars (b) where a1*b1 + ... + an*bn = 0
 *           or if the matrix (A) * b = 0
 *
 ****************************************/

#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>
#include <float.h>
#include <string.h>


#if MATH_CHECK_INDEXES
#include <assert.h>
#endif

namespace ap
{
template <typename T>
class Matrix3;

template <typename T>
class Vector3
{

public:
    T        x, y, z;

    // trivial ctor
    Vector3<T>() {
        x = y = z = 0;
    }

    // setting ctor
    Vector3<T>(const T x0, const T y0, const T z0) : x(x0), y(y0), z(z0) {
    }

    // function call operator
    void operator ()(const T x0, const T y0, const T z0)
    {
        x= x0; y= y0; z= z0;
    }

    // test for equality
    bool operator ==(const Vector3<T> &v) const;

    // test for inequality
    bool operator !=(const Vector3<T> &v) const;

    // negation
    Vector3<T> operator -(void) const;

    // addition
    Vector3<T> operator +(const Vector3<T> &v) const;

    // subtraction
    Vector3<T> operator -(const Vector3<T> &v) const;

    // uniform scaling
    Vector3<T> operator *(const T num) const;

    // uniform scaling
    Vector3<T> operator  /(const T num) const;

    // addition
    Vector3<T> &operator +=(const Vector3<T> &v);

    // subtraction
    Vector3<T> &operator -=(const Vector3<T> &v);

    // uniform scaling
    Vector3<T> &operator *=(const T num);

    // uniform scaling
    Vector3<T> &operator /=(const T num);

    // allow a vector3 to be used as an array, 0 indexed
    T & operator[](uint8_t i) {
        T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    // dot product
    T operator *(const Vector3<T> &v) const;

    // multiply a row vector by a matrix, to give a row vector
    Vector3<T> operator *(const Matrix3<T> &m) const;

    // multiply a column vector by a row vector, returning a 3x3 matrix
    Matrix3<T> mul_rowcol(const Vector3<T> &v) const;

    // cross product
    Vector3<T> operator %(const Vector3<T> &v) const;

    // computes the angle between this vector and another vector
    float angle(const Vector3<T> &v2) const;

    // check if all elements are zero
    bool is_zero(void) const { return (fabsf(x) < FLT_EPSILON) && (fabsf(y) < FLT_EPSILON) && (fabsf(z) < FLT_EPSILON); }


    // check if any elements are NAN
    bool is_nan(void);

    // rotate by a standard rotation
    void rotate(enum Rotation rotation);
    void rotate_inverse(enum Rotation rotation);

    // gets the length of this vector squared
    T  length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    float length(void) const;

    // normalizes this vector
    void normalize()
    {
        *this /= length();
    }

    // zero the vector
    void zero()
    {
        x = y = z = 0;
    }

    // returns the normalized version of this vector
    Vector3<T> normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void  reflect(const Vector3<T> &n)
    {
        Vector3<T>        orig(*this);
        project(n);
        *this = *this*2 - orig;
    }

    // projects this vector onto v
    void project(const Vector3<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    Vector3<T> projected(const Vector3<T> &v) const
    {
        return v * (*this * v)/(v*v);
    }


};

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<float>                  Vector3f;
typedef Vector3<double>                 Vector3d;

}; // namespace ap

#endif // VECTOR3_H
