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
    result += Math::GetSign(localDir[i]) * mScale[i] * mRotation.Basis(i);
  }
  return Vector3::cZero;
}

void ObbSupportShape::DebugDraw(const Vector4& color) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  DebugShape& shape = gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
  shape.Color(color);
  shape.SetTransform(transform);
}

//------------------------------------------------------------ Voronoi Region Tests
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  closestPoint = p0;
  return VoronoiRegion::Point0;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  float u, v;
  BarycentricCoordinates(q, p0, p1, u, v);

  if (u <= 0) {
    closestPoint = p0;
    return VoronoiRegion::Point0;
  }

  if (u >= 1) {
    closestPoint = p0;
    return VoronoiRegion::Point1;
  }

  closestPoint = u * p0 + v * p1;
  return VoronoiRegion::Edge01;
}

struct uv { float u; float v; };
uv p0p1;
uv p1p2;
uv p2p0;

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  BarycentricCoordinates(q, p0, p1, p0p1.u, p0p1.v); // line coords
  BarycentricCoordinates(q, p1, p2, p1p2.u, p1p2.v); // line coords
  BarycentricCoordinates(q, p2, p0, p2p0.u, p2p0.v); // line coords

  if (p0p1.v <= 0 && p2p0.u <= 0)
  {
    closestPoint = p0;
    return VoronoiRegion::Point0;
  }

  if (p1p2.v <= 0 && p0p1.u <= 0)
  {
    closestPoint = p1;
    return VoronoiRegion::Point1;
  }

  if (p2p0.v <= 0 && p1p2.u <= 0)
  {
    closestPoint = p2;
    return VoronoiRegion::Point2;
  }

  float u, v, w;
  BarycentricCoordinates(q, p0, p1, p2, u, v, w);    // tri  coords

  if (u > 0 && v > 0 && w > 0)
  {
    closestPoint = u * p0 + v * p1 + w * p2;
    return VoronoiRegion::Triangle012;
  }

  if (w < 0 && p0p1.u > 0 && p0p1.v > 0)
  {
    closestPoint = p0p1.u * p1 + p0p1.v * p0;
    return VoronoiRegion::Edge01;
  }

  if (u < 0 && p1p2.u > 0 && p1p2.v > 0)
  {
    closestPoint = p1p2.u * p2 + p1p2.v * p1;
    return VoronoiRegion::Edge12;
  }

  if (v < 0 && p2p0.u > 0 && p2p0.v > 0)
  {
    closestPoint = p2p0.u * p0 + p2p0.v * p2;
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
    return VoronoiRegion::Point0;
  }

  if (p0p1.u <= 0 && p1p2.v <= 0 && p1p3.v <= 0)
  {
    closestPoint = p1;
    return VoronoiRegion::Point1;
  }

  if (p1p2.u <= 0 && p0p2.u <= 0 && p2p3.v <= 0)
  {
    closestPoint = p2;
    return VoronoiRegion::Point2;
  }

  if (p0p3.u <= 0 && p2p3.u <= 0 && p1p3.u <= 0)
  {
    closestPoint = p3;
    return VoronoiRegion::Point3;
  }
  struct uvw { float u; float v; float w; }; // assuming u - 1st, v - 2nd, w - 3rd points. else swap u and w
  uvw p0p1p2, p0p2p3, p0p1p3, p1p2p3;
  BarycentricCoordinates(q, p0, p1, p2, p0p1p2.u, p0p1p2.v, p0p1p2.w);
  BarycentricCoordinates(q, p0, p2, p3, p0p2p3.u, p0p2p3.v, p0p2p3.w);
  BarycentricCoordinates(q, p0, p1, p3, p0p1p3.u, p0p1p3.v, p0p1p3.w);
  BarycentricCoordinates(q, p1, p2, p3, p1p2p3.u, p1p2p3.v, p1p2p3.w);

  if (p0p1.u > 0 && p0p1.v > 0 && p0p1p2.w < 0 && p0p1p3.w < 0)
  {
    closestPoint = p0p3.u * p3 + p0p3.v * p0;
    return VoronoiRegion::Edge01;
  }

  if (p0p2.u > 0 && p0p2.v > 0 && p0p1p2.v < 0 && p0p2p3.w < 0)
  {
    closestPoint = p0p3.u * p3 + p0p3.v * p0;
    return VoronoiRegion::Edge02;
  }

  if (p0p3.u > 0 && p0p3.v > 0 && p0p1p3.v < 0 && p0p2p3.v < 0)
  {
    closestPoint = p0p3.u * p3 + p0p3.v * p0;
    return VoronoiRegion::Edge03;
  }

  if (p1p2.u > 0 && p1p2.v > 0 && p0p1p2.u < 0 && p1p2p3.w < 0)
  {
    closestPoint = p1p2.u * p2 + p1p2.v * p1;
    return VoronoiRegion::Edge12;
  }

  if (p1p3.u > 0 && p1p3.v > 0 && p0p1p3.u < 0 && p1p2p3.v < 0)
  {
    closestPoint = p1p3.u * p3 + p1p3.v * p1;
    return VoronoiRegion::Edge13;
  }

  if (p2p3.u > 0 && p2p3.v > 0 && p0p2p3.u < 0 && p1p2p3.u < 0)
  {
    closestPoint = p2p3.u * p3 + p2p3.v * p2;
    return VoronoiRegion::Edge23;
  }


  if (p0p1p2.u > 0 && p0p1p2.v > 0 && p0p1p2.w > 0 && CheckNormal(q, p0, p1, p2, p3))
  {
    closestPoint = p0p1p2.u * p0 + p0p1p2.v * p3 + p0p1p2.w * p2;
    return VoronoiRegion::Triangle012;
  }

  if (p0p2p3.u > 0 && p0p2p3.v > 0 && p0p2p3.w > 0 && CheckNormal(q, p0, p2, p3, p1))
  {
    closestPoint = p0p2p3.u * p0 + p0p2p3.v * p2 + p0p2p3.w * p3;
    return VoronoiRegion::Triangle023;
  }

  if (p0p1p3.u > 0 && p0p1p3.v > 0 && p0p1p3.w > 0 && CheckNormal(q, p0, p1, p3, p2))
  {
    closestPoint = p0p1p3.u * p0 + p0p1p3.v * p1 + p0p1p3.w * p3;
    return VoronoiRegion::Triangle013;
  }

  if (p1p2p3.u > 0 && p1p2p3.v > 0 && p1p2p3.w > 0 && CheckNormal(q, p1, p2, p3, p0))
  {
    closestPoint = p1p2p3.u * p1 + p1p2p3.v * p2 + p1p2p3.w * p3;
    return VoronoiRegion::Triangle123;
  }

  closestPoint = q;
  return VoronoiRegion::Tetrahedra0123;
}

Gjk::Gjk()
{
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB, unsigned int maxIterations, CsoPoint& closestPoint, float epsilon, int debuggingIndex, bool debugDraw)
{
  Warn("Assignment5: Required function un-implemented");
  return false;
}

Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Vector3& direction)
{
  /******Student:Assignment5******/
  CsoPoint result;
  Warn("Assignment5: Required function un-implemented");

  return result;
}
