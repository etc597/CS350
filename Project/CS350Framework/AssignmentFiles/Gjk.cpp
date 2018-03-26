///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------SupportShape
Vector3 SupportShape::GetCenter(const std::vector<Vector3>& localPoints, const Matrix4& transform) const
{
  Vector3 center = Vector3::cZero;
  for (auto& pt : localPoints)
  {
    center += pt;
  }

  center /= (float)localPoints.size();
  return TransformPoint(transform, center);
}

Vector3 SupportShape::Support(const Vector3& worldDirection, const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform) const
{
  Vector3 result = Vector3::cZero;
  //Vector3 center = GetCenter(localPoints, Matrix4::cIdentity);
  Matrix4 worldToLocal = localToWorldTransform.Inverted();

  Vector3 locDir = TransformDirection(worldToLocal, worldDirection);

  float furthest = Math::NegativeMin();
  for (auto& pt : localPoints)
  {
    float dist = Math::Dot(locDir, pt);// -center);
    if (dist > furthest)
    {
      furthest = dist;
      result = pt;
    }
  }

  return Math::TransformPoint(localToWorldTransform, result);
}

void SupportShape::DebugDraw(const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform, const Vector4& color) const
{
  for (auto& pt : localPoints)
  {
    gDebugDrawer->DrawPoint(pt).SetTransform(localToWorldTransform).Color(color);
  }
}

//-----------------------------------------------------------------------------ModelSupportShape
Vector3 ModelSupportShape::GetCenter() const
{
  return SupportShape::GetCenter(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

Vector3 ModelSupportShape::Support(const Vector3& worldDirection) const
{
  return SupportShape::Support(worldDirection, mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

void ModelSupportShape::DebugDraw(const Vector4& color) const
{
  SupportShape::DebugDraw(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

//-----------------------------------------------------------------------------PointsSupportShape
PointsSupportShape::PointsSupportShape()
{
  mScale = Vector3(1);
  mRotation = Matrix3::cIdentity;
  mTranslation = Vector3::cZero;
}

Vector3 PointsSupportShape::GetCenter() const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  return SupportShape::GetCenter(mLocalSpacePoints, transform);
}

Vector3 PointsSupportShape::Support(const Vector3& worldDirection) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  return SupportShape::Support(worldDirection, mLocalSpacePoints, transform);
}

void PointsSupportShape::DebugDraw(const Vector4& color) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  SupportShape::DebugDraw(mLocalSpacePoints, transform, color);
}

//-----------------------------------------------------------------------------SphereSupportShape
Vector3 SphereSupportShape::GetCenter() const
{
  return mSphere.mCenter;
}

Vector3 SphereSupportShape::Support(const Vector3& worldDirection) const
{
  Vector3 normalDir = worldDirection.AttemptNormalized();
  return GetCenter() + mSphere.mRadius * normalDir;
}

void SphereSupportShape::DebugDraw(const Vector4& color) const
{
  DebugShape& shape = gDebugDrawer->DrawSphere(mSphere);
  shape.Color(color);
}

//-----------------------------------------------------------------------------ObbSupportShape
Vector3 ObbSupportShape::GetCenter() const
{
  return mTranslation;
}

Vector3 ObbSupportShape::Support(const Vector3& worldDirection) const
{
  Vector3 result = GetCenter();
  Vector3 localDir = Math::Transform(mRotation.Inverted(), worldDirection);
  for (auto i = 0; i < 3; ++i)
  {
    result += Math::GetSign(localDir[i]) * 0.5f * mScale[i] * mRotation.Basis(i);
  }
  return result;
}

void ObbSupportShape::DebugDraw(const Vector4& color) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  DebugShape& shape = gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
  shape.Color(color);
  shape.SetTransform(transform);
}

//------------------------------------------------------------ Voronoi Region Tests
struct uv { float u; float v; };
struct uvw { float u; float v; float w; }; // assuming u - 1st, v - 2nd, w - 3rd points. else swap u and w

Vector3 ConstructPoint(float u, float v, const Vector3& a, const Vector3& b)
{
  return u * a + b * v;
}
Vector3 ConstructPoint(float u, float v, float w, const Vector3& a, const Vector3& b, const Vector3& c)
{
  return ConstructPoint(u, v, a, b) + w * c;
}

Vector3 ConstructPoint(uv uv, const Vector3& a, const Vector3& b)
{
  return ConstructPoint(uv.u, uv.v, a, b);
}

Vector3 ConstructPoint(uvw uvw, const Vector3& a, const Vector3& b, const Vector3& c)
{
  return ConstructPoint(uvw.u, uvw.v, uvw.w, a, b, c);
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  closestPoint = p0;
  searchDirection = q - closestPoint;
  newSize = 1;
  newIndices[0] = 0;
  return VoronoiRegion::Point0;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  float u, v;
  BarycentricCoordinates(q, p0, p1, u, v);

  if (v <= 0) {
    closestPoint = p0;
    searchDirection = q - closestPoint;
    newSize = 1;
    newIndices[0] = 0;
    return VoronoiRegion::Point0;
  }

  if (v >= 1) {
    closestPoint = p1;
    searchDirection = q - closestPoint;
    newSize = 1;
    newIndices[0] = 1;
    return VoronoiRegion::Point1;
  }

  closestPoint = ConstructPoint(u, v, p0, p1);
  searchDirection = q - closestPoint;
  newSize = 2;
  newIndices[0] = 0;
  newIndices[1] = 1;
  return VoronoiRegion::Edge01;
}



VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  uv p0p1;
  uv p1p2;
  uv p2p0;

  BarycentricCoordinates(q, p0, p1, p0p1.u, p0p1.v); // line coords
  BarycentricCoordinates(q, p1, p2, p1p2.u, p1p2.v); // line coords
  BarycentricCoordinates(q, p2, p0, p2p0.u, p2p0.v); // line coords

  if (p0p1.v <= 0 && p2p0.u <= 0)
  {
    closestPoint = p0;
    searchDirection = q - closestPoint;
    newSize = 1;
    newIndices[0] = 0;
    return VoronoiRegion::Point0;
  }

  if (p1p2.v <= 0 && p0p1.u <= 0)
  {
    closestPoint = p1;
    searchDirection = q - closestPoint;
    newSize = 1;
    newIndices[0] = 1;
    return VoronoiRegion::Point1;
  }

  if (p2p0.v <= 0 && p1p2.u <= 0)
  {
    closestPoint = p2;
    searchDirection = q - closestPoint;
    newSize = 1;
    newIndices[0] = 2;
    return VoronoiRegion::Point2;
  }

  float u, v, w;
  BarycentricCoordinates(q, p0, p1, p2, u, v, w);    // tri  coords

  if (u > 0 && v > 0 && w > 0)
  {
    closestPoint = ConstructPoint(u, v, w, p0, p1, p2);
    searchDirection = q - closestPoint;
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 1;
    newIndices[2] = 2;
    return VoronoiRegion::Triangle012;
  }

  if (w < 0 && p0p1.u > 0 && p0p1.v > 0)
  {
    closestPoint = ConstructPoint(p0p1, p0, p1);
    searchDirection = q - closestPoint;
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 1;
    return VoronoiRegion::Edge01;
  }

  if (u < 0 && p1p2.u > 0 && p1p2.v > 0)
  {
    closestPoint = ConstructPoint(p1p2, p1, p2);
    searchDirection = q - closestPoint;
    newSize = 2;
    newIndices[0] = 1;
    newIndices[1] = 2;
    return VoronoiRegion::Edge12;
  }

  if (v < 0 && p2p0.u > 0 && p2p0.v > 0)
  {
    closestPoint = ConstructPoint(p2p0, p2, p0);
    searchDirection = q - closestPoint;
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 2;
    return VoronoiRegion::Edge02;
  }

  return VoronoiRegion::Unknown;
}

bool CheckNormal(const Vector3& q, const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d)
{
  // d - interior point. a,b,c triangle surface
  Vector3 normal = Math::Cross(b - a, c - a);
  if (Math::Dot(normal, d - a) > 0)
    normal *= -1;
  return Math::Dot(normal, q - a) > 0;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  uv p0p1, p1p2, p0p2;
  uv p0p3, p2p3, p1p3;
  BarycentricCoordinates(q, p0, p1, p0p1.u, p0p1.v);
  BarycentricCoordinates(q, p1, p2, p1p2.u, p1p2.v);
  BarycentricCoordinates(q, p0, p2, p0p2.u, p0p2.v);
  BarycentricCoordinates(q, p0, p3, p0p3.u, p0p3.v);
  BarycentricCoordinates(q, p2, p3, p2p3.u, p2p3.v);
  BarycentricCoordinates(q, p1, p3, p1p3.u, p1p3.v);

  if (p0p1.v <= 0 && p0p2.v <= 0 && p0p3.v <= 0)
  {
    closestPoint = p0;
    searchDirection = q - closestPoint;
    newSize = 1;
    newIndices[0] = 0;
    return VoronoiRegion::Point0;
  }

  if (p0p1.u <= 0 && p1p2.v <= 0 && p1p3.v <= 0)
  {
    closestPoint = p1;
    searchDirection = q - closestPoint;
    newSize = 1;
    newIndices[0] = 1;
    return VoronoiRegion::Point1;
  }

  if (p1p2.u <= 0 && p0p2.u <= 0 && p2p3.v <= 0)
  {
    closestPoint = p2;
    searchDirection = q - closestPoint;
    newSize = 1;
    newIndices[0] = 2;
    return VoronoiRegion::Point2;
  }

  if (p0p3.u <= 0 && p2p3.u <= 0 && p1p3.u <= 0)
  {
    closestPoint = p3;
    searchDirection = q - closestPoint;
    newSize = 1;
    newIndices[0] = 3;
    return VoronoiRegion::Point3;
  }

  uvw p0p1p2, p0p2p3, p0p1p3, p1p2p3;
  BarycentricCoordinates(q, p0, p1, p2, p0p1p2.u, p0p1p2.v, p0p1p2.w);
  BarycentricCoordinates(q, p0, p2, p3, p0p2p3.u, p0p2p3.v, p0p2p3.w);
  BarycentricCoordinates(q, p0, p1, p3, p0p1p3.u, p0p1p3.v, p0p1p3.w);
  BarycentricCoordinates(q, p1, p2, p3, p1p2p3.u, p1p2p3.v, p1p2p3.w);

  if (p0p1.u > 0 && p0p1.v > 0 && p0p1p2.w < 0 && p0p1p3.w < 0)
  {
    closestPoint = ConstructPoint(p0p1, p0, p1);
    searchDirection = q - closestPoint;
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 1;
    return VoronoiRegion::Edge01;
  }

  if (p0p2.u > 0 && p0p2.v > 0 && p0p1p2.v < 0 && p0p2p3.w < 0)
  {
    closestPoint = ConstructPoint(p0p2, p0, p2);
    searchDirection = q - closestPoint;
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 2;
    return VoronoiRegion::Edge02;
  }

  if (p0p3.u > 0 && p0p3.v > 0 && p0p1p3.v < 0 && p0p2p3.v < 0)
  {
    closestPoint = ConstructPoint(p0p3, p0, p3);
    searchDirection = q - closestPoint;
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 3;
    return VoronoiRegion::Edge03;
  }

  if (p1p2.u > 0 && p1p2.v > 0 && p0p1p2.u < 0 && p1p2p3.w < 0)
  {
    closestPoint = ConstructPoint(p1p2, p1, p2);
    searchDirection = q - closestPoint;
    newSize = 2;
    newIndices[0] = 1;
    newIndices[1] = 2;
    return VoronoiRegion::Edge12;
  }

  if (p1p3.u > 0 && p1p3.v > 0 && p0p1p3.u < 0 && p1p2p3.v < 0)
  {
    closestPoint = ConstructPoint(p1p3, p1, p3);
    searchDirection = q - closestPoint;
    newSize = 2;
    newIndices[0] = 1;
    newIndices[1] = 3;
    return VoronoiRegion::Edge13;
  }

  if (p2p3.u > 0 && p2p3.v > 0 && p0p2p3.u < 0 && p1p2p3.u < 0)
  {
    closestPoint = ConstructPoint(p2p3, p2, p3);
    searchDirection = q - closestPoint;
    newSize = 2;
    newIndices[0] = 2;
    newIndices[1] = 3;
    return VoronoiRegion::Edge23;
  }


  if (p0p1p2.u > 0 && p0p1p2.v > 0 && p0p1p2.w > 0 && CheckNormal(q, p0, p1, p2, p3))
  {
    closestPoint = ConstructPoint(p0p1p2, p0, p1, p2);
    searchDirection = q - closestPoint;
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 1;
    newIndices[2] = 2;
    return VoronoiRegion::Triangle012;
  }

  if (p0p2p3.u > 0 && p0p2p3.v > 0 && p0p2p3.w > 0 && CheckNormal(q, p0, p2, p3, p1))
  {
    closestPoint = ConstructPoint(p0p2p3, p0, p2, p3);
    searchDirection = q - closestPoint;
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 2;
    newIndices[2] = 3;
    return VoronoiRegion::Triangle023;
  }

  if (p0p1p3.u > 0 && p0p1p3.v > 0 && p0p1p3.w > 0 && CheckNormal(q, p0, p1, p3, p2))
  {
    closestPoint = ConstructPoint(p0p1p3, p0, p1, p3);
    searchDirection = q - closestPoint;
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 1;
    newIndices[2] = 3;
    return VoronoiRegion::Triangle013;
  }

  if (p1p2p3.u > 0 && p1p2p3.v > 0 && p1p2p3.w > 0 && CheckNormal(q, p1, p2, p3, p0))
  {
    closestPoint = ConstructPoint(p1p2p3, p1, p2, p3);
    searchDirection = q - closestPoint;
    newSize = 3;
    newIndices[0] = 1;
    newIndices[1] = 2;
    newIndices[2] = 3;
    return VoronoiRegion::Triangle123;
  }

  closestPoint = q;
  searchDirection = q - closestPoint;
  newSize = 4;
  newIndices[0] = 0;
  newIndices[1] = 1;
  newIndices[2] = 2;
  newIndices[3] = 3;

  return VoronoiRegion::Tetrahedra0123;
}

Gjk::Gjk()
{
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB, unsigned int maxIterations, CsoPoint& closestPoint, float epsilon, int debuggingIndex, bool debugDraw)
{
  unsigned iter = 0;
  Vector3 searchDir = shapeB->GetCenter() - shapeA->GetCenter();
  
  int indices[4] = { 0 };
  size_t size = 1;
  Vector3 Q = Vector3::cZero; // origin
  closestPoint = ComputeSupport(shapeA, shapeB, searchDir);
  Vector3 P = closestPoint.mCsoPoint;
  searchDir = Q - P;
  CsoPoint simplex[4] = { closestPoint };

  while (iter < maxIterations)
  {
    switch (size)
    {
    case 1:
      IdentifyVoronoiRegion(Q, simplex[indices[0]].mCsoPoint, size, indices, P, searchDir);
      break;
    case 2:
      IdentifyVoronoiRegion(Q, simplex[indices[0]].mCsoPoint, simplex[indices[1]].mCsoPoint, size, indices, P, searchDir);
      break;
    case 3:
      IdentifyVoronoiRegion(Q, simplex[indices[0]].mCsoPoint, simplex[indices[1]].mCsoPoint, simplex[indices[2]].mCsoPoint, size, indices, P, searchDir);
      break;
    case 4:
      IdentifyVoronoiRegion(Q, simplex[indices[0]].mCsoPoint, simplex[indices[1]].mCsoPoint, simplex[indices[2]].mCsoPoint, simplex[indices[3]].mCsoPoint, size, indices, P, searchDir);
      break;
    }

    if (P == Q)
    {
      return true;
    }

    auto newPoint = ComputeSupport(shapeA, shapeB, searchDir);

    // if Q - newPoint * search < Q - P * search, terminate, also some other early out conditions
    if (Math::Distance(P - newPoint.mCsoPoint, searchDir) <= epsilon)
    {
      break;
    }

    // add new point to simplex and repeat
    indices[size] = GetFreeIndex(indices, size);
    simplex[indices[size]] = newPoint;
    ++size;

    ++iter;
  }

  ReconstructPoint(P, closestPoint, indices, simplex, size);
  return false;
}

Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Vector3& direction)
{
  CsoPoint result;
  result.mPointA = shapeA->Support(direction);
  result.mPointB = shapeB->Support(-direction);
  result.mCsoPoint = result.mPointA - result.mPointB;
  return result;
}

int Gjk::GetFreeIndex(int indices[4], size_t size)
{

  for (auto i = 0; i < 4; ++i)
  {
    bool found = false;
    for (size_t j = 0; j < size; ++j)
    {
      if (indices[j] == i) { found = true;  break; }
    }
    if (!found) return i;
  }
  return -1;
}

void Gjk::ReconstructPoint(const Vector3& P, CsoPoint & closestPoint, int indices[4], CsoPoint simplex[4], size_t size)
{
  switch (size - 1)
  {
  case 1:
    closestPoint = simplex[indices[0]];
    break;
  case 2:
  {
    float u, v;
    BarycentricCoordinates(P, simplex[indices[0]].mCsoPoint, simplex[indices[1]].mCsoPoint, u, v);
    closestPoint.mCsoPoint = P;
    closestPoint.mPointA = ConstructPoint(u, v, simplex[indices[0]].mPointA, simplex[indices[1]].mPointA);
    closestPoint.mPointB = ConstructPoint(u, v, simplex[indices[0]].mPointB, simplex[indices[1]].mPointB);
  }
    break;
  case 3:
  {
    float u, v, w;
    BarycentricCoordinates(P, simplex[indices[0]].mCsoPoint, simplex[indices[1]].mCsoPoint, simplex[indices[2]].mCsoPoint, u, v, w);
    closestPoint.mCsoPoint = P;
    closestPoint.mPointA = ConstructPoint(u, v, w, simplex[indices[0]].mPointA, simplex[indices[1]].mPointA, simplex[indices[2]].mPointA);
    closestPoint.mPointB = ConstructPoint(u, v, w, simplex[indices[0]].mPointB, simplex[indices[1]].mPointB, simplex[indices[2]].mPointB);
  }
    break;
  case 4:
    // idk yet
    break;
  }
}
