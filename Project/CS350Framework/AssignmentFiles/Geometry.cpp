///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

bool outRange(float val, float minRange, float maxRange)
{
  return val < minRange || val > maxRange;
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b,
                            float& u, float& v, float expansionEpsilon)
{
  Vector3 BA = a - b;
  Vector3 BP = point - b;

  float denom = Math::Dot(BA, BA);

  if (denom == 0) {
    u = v = 0;
    return false;
  }

  u = Math::Dot(BA, BP) / denom;
  v = 1 - u;

  float minRange = -expansionEpsilon;
  float maxRange = 1 + expansionEpsilon;

  if (outRange(u, minRange, maxRange)
    || outRange(v, minRange, maxRange)) {
    return false;
  }

  return true;
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b, const Vector3& c,
                            float& u, float& v, float& w, float expansionEpsilon)
{
  Vector3 Npbc = Cross(b - point, c - point);
  Vector3 Npca = Cross(c - point, a - point);
  Vector3 N = Cross(b - a, c - a);

  float area = Math::Dot(N, N);

  if (area == 0) {
    u = v = w = 0;
    return false;
  }

  u = Math::Dot(N, Npbc) / area;
  v = Math::Dot(N, Npca) / area;
  w = 1 - u - v;

  float minRange = -expansionEpsilon;
  float maxRange = 1 + expansionEpsilon;

  if (outRange(u, minRange, maxRange)
    || outRange(v, minRange, maxRange)
    || outRange(w, minRange, maxRange)) {
    return false;
  }

  return true;
}

IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane, float epsilon)
{
  Vector4 p = Vector4(point.x, point.y, point.z, -1);
  float w = Math::Dot(plane, p);
  if (w > epsilon) {
    return IntersectionType::Inside;
  }
  else if (w < -epsilon) {
    return IntersectionType::Outside;
  }
  return IntersectionType::Coplanar;
}

bool PointSphere(const Vector3& point, const Vector3& sphereCenter, float sphereRadius)
{
  Vector3 d = sphereCenter - point;
  return Math::Dot(d, d) - sphereRadius * sphereRadius <= 0;
}

bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax)
{
  for (int i = 0; i < 3; ++i) {
    if (outRange(point[i], aabbMin[i], aabbMax[i])) {
      return false;
    }
  }
  return true;
}

bool RayPlane(const Vector3& rayStart, const Vector3& rayDir,
              const Vector4& plane, float& t, float parallelCheckEpsilon)
{
  ++Application::mStatistics.mRayPlaneTests;
  
  Vector3 normal = Vector3(plane.x, plane.y, plane.z);
  float denom = Math::Dot(normal, rayDir);

  if (denom > -parallelCheckEpsilon && denom < parallelCheckEpsilon) {
    return false;
  }

  float num = plane.w - Math::Dot(normal, rayStart);
  t = num / denom;
  if (t < 0) {
    return false;
  }

  return true;
}

bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
                 const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                 float& t, float triExpansionEpsilon, float parallelCheckEpsilon)
{
  ++Application::mStatistics.mRayTriangleTests;

  Plane trianglePlane;
  trianglePlane.Set(triP0, triP1, triP2);

  if (!RayPlane(rayStart, rayDir, trianglePlane.mData, t, parallelCheckEpsilon)) {
    return false;
  }

  Vector3 point = rayStart + rayDir * t;

  float u, v, w;
  return BarycentricCoordinates(point, triP0, triP1, triP2, u, v, w, parallelCheckEpsilon);
}

bool RaySphere(const Vector3& rayStart, const Vector3& rayDir,
               const Vector3& sphereCenter, float sphereRadius,
               float& t)
{
  ++Application::mStatistics.mRaySphereTests;

  Vector3 m = sphereCenter - rayStart;
  float a = Math::Dot(rayDir, rayDir);
  float b = -2 * Math::Dot(m, rayDir);
  float c = Math::Dot(m, m) - sphereRadius * sphereRadius;

  float discr = b * b - 4 * a * c; // discriminant

  if (discr < 0) {
    return false;
  }

  if (discr == 0) {
    t = -b / (2 * a);
    if (t < 0) {
      return false;
    }

    return true;
  }
  
  float t1 = (-b + Math::Sqrt(discr)) / (2 * a);
  float t2 = (-b - Math::Sqrt(discr)) / (2 * a);

  if (t1 < 0 && t2 < 0) {
    return false;
  }

  // if product is negative, one is + and other is -
  if (t1 * t2 <= 0) {
    t = 0;
    return true;
  }

  t = Math::Min(t1, t2);

  return true;
}

bool RayAabb(const Vector3& rayStart, const Vector3& rayDir,
             const Vector3& aabbMin, const Vector3& aabbMax, float& t, float parallelCheckEpsilon)
{
  ++Application::mStatistics.mRayAabbTests;
  float tmin = 0;
  float tmax = 0;
  bool first = true;

  for (int i = 0; i < 3; ++i) {
    if (rayDir[i] == 0) {
      if (rayStart[i] < aabbMin[i] || rayStart[i] > aabbMax[i]) {
        return false;
      }
      continue;
    }
    float min = (aabbMin[i] - rayStart[i]) / rayDir[i];
    float max = (aabbMax[i] - rayStart[i]) / rayDir[i];
    
    if (rayDir[i] < 0) {
      Math::Swap(min, max);
    }

    first ? tmin = min : tmin = Math::Max(min, tmin);
    first ? tmax = max : tmax = Math::Min(max, tmax);

    first = false;
  }

  if (tmin > tmax) {
    return false;
  }

  if (tmin * tmax <= 0) {
    t = 0;
    return true;
  }

  t = tmin;

  if (t < 0) {
    return false;
  }

  return true;
}

IntersectionType::Type PlaneTriangle(const Vector4& plane, 
                                     const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                                     float epsilon)
{
  ++Application::mStatistics.mPlaneTriangleTests;

  return static_cast<IntersectionType::Type>(PointPlane(triP0, plane, epsilon)
    | PointPlane(triP1, plane, epsilon)
    | PointPlane(triP2, plane, epsilon));
}

IntersectionType::Type PlaneSphere(const Vector4& plane,
                                   const Vector3& sphereCenter, float sphereRadius)
{
  ++Application::mStatistics.mPlaneSphereTests;

  IntersectionType::Type type = PointPlane(sphereCenter, plane, sphereRadius);

  if (type == IntersectionType::Coplanar) {
    return IntersectionType::Overlaps;
  }

  return type;
}

IntersectionType::Type PlaneAabb(const Vector4& plane,
                                 const Vector3& aabbMin, const Vector3& aabbMax)
{
  ++Application::mStatistics.mPlaneAabbTests;

  Vector3 c = aabbMin;

  float r = 0;
  for (int i = 0; i < 3; ++i) {
    float e = (aabbMax[i] - aabbMin[i]) * 0.5f;
    r += e * Math::Abs(plane[i]);
    c[i] += e;
  }

  return PlaneSphere(plane, c, r);
}

IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
                                       const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                                       float epsilon)
{
  ++Application::mStatistics.mFrustumTriangleTests;

  IntersectionType::Type intersection = IntersectionType::Inside;

  for (int i = 0; i < 6; ++i) {
    IntersectionType::Type type = PlaneTriangle(planes[i], triP0, triP1, triP2, epsilon);
    if (type == IntersectionType::Outside) {
      return type;
    }
    else if (type != IntersectionType::Inside) {
      intersection = type;
    }
  }

  if (intersection != IntersectionType::Inside) {
    return IntersectionType::Overlaps;
  }
  return IntersectionType::Inside;
}

IntersectionType::Type FrustumSphere(const Vector4 planes[6],
                                     const Vector3& sphereCenter, float sphereRadius, size_t& lastAxis)
{
  ++Application::mStatistics.mFrustumSphereTests;
  
  IntersectionType::Type intersection = IntersectionType::Inside;

  for (int i = 0; i < 6; ++i) {
    size_t current = (lastAxis + i) % 6;
    IntersectionType::Type type = PlaneSphere(planes[current], sphereCenter, sphereRadius);
    if (type == IntersectionType::Outside) {
      return type;
    }
    else if (type != IntersectionType::Inside) {
      intersection = type;
    }
  }

  if (intersection != IntersectionType::Inside) {
    return IntersectionType::Overlaps;
  }
  return IntersectionType::Inside;
}

IntersectionType::Type FrustumAabb(const Vector4 planes[6],
                                   const Vector3& aabbMin, const Vector3& aabbMax, size_t& lastAxis)
{
  ++Application::mStatistics.mFrustumAabbTests;

  IntersectionType::Type intersection = IntersectionType::Inside;

  for (int i = 0; i < 6; ++i) {
    size_t current = (lastAxis + i) % 6;
    IntersectionType::Type type = PlaneAabb(planes[current], aabbMin, aabbMax);
    if (type == IntersectionType::Outside) {
      return type;
    }
    else if (type != IntersectionType::Inside) {
      intersection = type;
    }
  }

  if (intersection != IntersectionType::Inside) {
    return IntersectionType::Overlaps;
  }
  return IntersectionType::Inside;
}

bool SphereSphere(const Vector3& sphereCenter0, float sphereRadius0,
                  const Vector3& sphereCenter1, float sphereRadius1)
{
  ++Application::mStatistics.mSphereSphereTests;

  return PointSphere(sphereCenter0, sphereCenter1, sphereRadius0 + sphereRadius1);
}

bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0,
              const Vector3& aabbMin1, const Vector3& aabbMax1)
{
  ++Application::mStatistics.mAabbAabbTests;

  for (int i = 0; i < 3; ++i) {
    if (aabbMin0[i] > aabbMax1[i]
      || aabbMin1[i] > aabbMax0[i]) {
      return false;
    }
  }

  return true;
}
