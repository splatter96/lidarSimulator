//[header]
// This program illustrates how the concept of vector and matrix can be implemented
// in C++. This is a light version of the implementation. It contains the most
// essential methods to manipulate vectors and matrices. It should be enough
// for most projects. Vectors and matrices are really the alphabet as we said
// in the lesson of any graphics application. It's really important you feel
// confortable with these techniques especially with the concepts of
// normalizing vectors, computing their length, computing the dot and cross products
// of two vectors, and the point- and vector-matrix multiplication (and knowing
// the difference between the two).
//[/header]
//[compile]
// c++ geometry.cpp  -o geometry -std=c++11
//[/compile]
//[ignore]
// Copyright (C) 2012  www.scratchapixel.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//[/ignore]
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <cmath>

template<typename T>
class Vec2
{
public:
    Vec2() : x(0), y(0) {}
    Vec2(T xx) : x(xx), y(xx) {}
    Vec2(T xx, T yy) : x(xx), y(yy) {}
    Vec2 operator + (const Vec2 &v) const
    { return Vec2(x + v.x, y + v.y); }
    Vec2 operator / (const T &r) const
    { return Vec2(x / r, y / r); }
    Vec2 operator * (const T &r) const
    { return Vec2(x * r, y * r); }
    Vec2& operator /= (const T &r)
    { x /= r, y /= r; return *this; }
    Vec2& operator *= (const T &r)
    { x *= r, y *= r; return *this; }
    friend std::ostream& operator << (std::ostream &s, const Vec2<T> &v)
    {
        return s << '[' << v.x << ' ' << v.y << ']';
    }
    friend Vec2 operator * (const T &r, const Vec2<T> &v)
    { return Vec2(v.x * r, v.y * r); }
    T x, y;
};

typedef Vec2<float> Vec2f;

/**
 * Class to represent a single Triangle
 */
class Triangle
{
    public:
        Triangle() : normal(tf::Vector3()), v1(tf::Vector3()), v2(tf::Vector3()), v3(tf::Vector3()) {}
        Triangle(tf::Vector3 normalp, tf::Vector3 v1p, tf::Vector3 v2p, tf::Vector3 v3p) :
            normal(normalp), v1(v1p), v2(v2p), v3(v3p) {}
        tf::Vector3 normal;
        tf::Vector3 v1;
        tf::Vector3 v2;
        tf::Vector3 v3;
};