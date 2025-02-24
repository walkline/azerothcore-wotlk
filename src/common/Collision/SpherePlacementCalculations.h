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

#ifndef SPHEREPLACEMENTCALCULATIONS_H
#define SPHEREPLACEMENTCALCULATIONS_H

#include <G3D/Vector3.h>
#include "Triangle.h"

inline float dot(const G3D::Vector3& a, const G3D::Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline G3D::Vector3 closestPointOnTriangle(const G3D::Vector3& p, const Triangle& tri) {
    // Rename triangle vertices for clarity.
    const G3D::Vector3& A = tri.a;
    const G3D::Vector3& B = tri.b;
    const G3D::Vector3& C = tri.c;

    // Compute vectors for two edges sharing vertex A.
    G3D::Vector3 AB = B - A;
    G3D::Vector3 AC = C - A;
    // Vector from vertex A to point p.
    G3D::Vector3 AP = p - A;

    // Compute dot products to measure how far p projects onto the triangle edges.
    float d1 = dot(AB, AP); // p's projection on AB
    float d2 = dot(AC, AP); // p's projection on AC

    // Check if p is in the vertex region outside A.
    if (d1 <= 0.f && d2 <= 0.f)
        return A;

    // Check if p is in the vertex region outside B.
    G3D::Vector3 BP = p - B;
    float d3 = dot(AB, BP); // p's projection on AB from B's perspective
    float d4 = dot(AC, BP); // p's projection on AC (relative to A)
    if (d3 >= 0.f && d4 <= d3)
        return B;

    // Check if p is in the edge region of AB.
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
        return A + AB * (d1 / (d1 - d3));

    // Check if p is in the vertex region outside C.
    G3D::Vector3 CP = p - C;
    float d5 = dot(AB, CP); // p's projection on AB (relative to C)
    float d6 = dot(AC, CP); // p's projection on AC from C's perspective
    if (d6 >= 0.f && d5 <= d6)
        return C;

    // Check if p is in the edge region of AC.
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
        return A + AC * (d2 / (d2 - d6));

    // Check if p is in the edge region of BC.
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
        return B + (C - B) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

    // p is inside the face region. Compute barycentric coordinates (v, w)
    // so that p can be expressed as: A + AB*v + AC*w.
    float denom = 1.f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return A + AB * v + AC * w;
}

inline float length(const G3D::Vector3& v) {
    return sqrt(dot(v, v));
}
inline G3D::Vector3 normalize(const G3D::Vector3& v) {
    float len = length(v);
    return (len > 0.f) ? v / len : G3D::Vector3(0, 0, 0);
}

inline G3D::Vector3 resolveCollisionWithSphere(G3D::Vector3 pos, float radius, const std::unordered_set<Triangle, TriangleHasher> &triangles) {
    bool collisionFound = true;
    constexpr int maxIterations = 10;
    int iterations = 0;

    // Loop until no collisions are found or the maximum number of iterations is reached.
    while (collisionFound && iterations < maxIterations) {
        collisionFound = false;
        // Iterate over each triangle in the scene.
        for (const auto& tri : triangles) {
            // Find the closest point on the triangle to the sphere center.
            G3D::Vector3 closestPoint = closestPointOnTriangle(pos, tri);
            // Compute the vector from this closest point to the sphere center.
            G3D::Vector3 difference = pos - closestPoint;
            float distanceSquared = dot(difference, difference);
            // If the squared distance is less than the squared radius, a collision occurs.
            if (distanceSquared < radius * radius) {
                const float distance = sqrt(distanceSquared);
                const float penetrationDepth = radius - distance;
                G3D::Vector3 pushDirection;
                // If the sphere center is not exactly at the collision point, normalize the difference vector.
                if (distance > 0.0001f)
                    pushDirection = difference / distance;
                else {
                    // If the sphere center is exactly on the triangle, use the triangle's normal.
                    G3D::Vector3 triangleNormal = normalize((tri.b - tri.a).cross(tri.b - tri.a));
                    // Use world up (0,0,1) if the triangle normal is degenerate.
                    pushDirection = (length(triangleNormal) > 0.0001f) ? triangleNormal : G3D::Vector3(0, 0, 1);
                }
                // Move the sphere center by the penetration depth along the push direction.
                pos = pos + pushDirection * penetrationDepth;
                collisionFound = true;
            }
        }
        iterations++;
    }
    return pos;
}

inline bool isPointInsideSphere(const G3D::Vector3& point, const G3D::Vector3& sphereCenter, float sphereRadiusSq)
{
    return (point - sphereCenter).squaredLength() <= sphereRadiusSq;
}

inline bool doesEdgeIntersectSphere(const G3D::Vector3& p1, const G3D::Vector3& p2, const G3D::Vector3& sphereCenter, float sphereRadiusSq)
{
    G3D::Vector3 dir = p2 - p1;
    G3D::Vector3 centerToP1 = p1 - sphereCenter;

    float a = dir.squaredLength();
    float b = 2.0f * centerToP1.dot(dir);
    float c = centerToP1.squaredLength() - sphereRadiusSq;

    float discriminant = b * b - 4.0f * a * c;
    return discriminant >= 0;
}

inline bool isTriangleInsideOrIntersectingSphere(const G3D::Vector3& v0, const G3D::Vector3& v1, const G3D::Vector3& v2,
                                                 const G3D::Vector3& sphereCenter, float sphereRadiusSq)
{
    return isPointInsideSphere(v0, sphereCenter, sphereRadiusSq) ||
           isPointInsideSphere(v1, sphereCenter, sphereRadiusSq) ||
           isPointInsideSphere(v2, sphereCenter, sphereRadiusSq) ||
           doesEdgeIntersectSphere(v0, v1, sphereCenter, sphereRadiusSq) ||
           doesEdgeIntersectSphere(v1, v2, sphereCenter, sphereRadiusSq) ||
           doesEdgeIntersectSphere(v2, v0, sphereCenter, sphereRadiusSq);
}

#endif //SPHEREPLACEMENTCALCULATIONS_H
