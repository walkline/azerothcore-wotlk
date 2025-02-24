/*
 * This file is part of the AzerothCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Affero General Public License as published by the
 * Free Software Foundation; either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <G3D/Vector3.h>

struct Triangle
{
    Triangle(G3D::Vector3 a, G3D::Vector3 b, G3D::Vector3 c) : a(a), b(b), c(c) {};

    G3D::Vector3 a;
    G3D::Vector3 b;
    G3D::Vector3 c;

    bool operator==(const Triangle& other) const
    {
        return a == other.a && b == other.b && c == other.c;
    }
};

struct TriangleHasher {
    std::size_t operator()(const Triangle& t) const {
        std::hash<float> hasher;
        return hasher(t.a.x) ^ (hasher(t.a.y) << 1) ^ (hasher(t.a.z) << 2) ^
               (hasher(t.b.x) << 3) ^ (hasher(t.b.y) << 4) ^ (hasher(t.b.z) << 5) ^
               (hasher(t.c.x) << 6) ^ (hasher(t.c.y) << 7) ^ (hasher(t.c.z) << 8);
    }
};

#endif //TRIANGLE_H
